using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>
/// Removes a known I/Q gain and phase imbalance before demodulation. The imbalance follows the
/// convention of the hardware characterisation (neohw's mixer check, mixer.json): samples are
/// z = I − jQ, the gain error is I over Q in dB, and the phase error is the departure of Q from
/// quadrature, so the receiver delivers Q' = (Q·cos φ − I·sin φ) / g. Correction recovers
/// Q = (g·Q' + I·sin φ) / cos φ and leaves I unchanged.
/// </summary>
public sealed class IqCorrector
{
    public const double MaxGainDb = 6.0;
    public const double MaxPhaseDeg = 30.0;

    private readonly double _gain;
    private readonly double _sin;
    private readonly double _cos;

    public IqCorrector(double gainDb, double phaseDeg)
    {
        if (!double.IsFinite(gainDb) || Math.Abs(gainDb) > MaxGainDb)
            throw new ArgumentException($"I/Q gain error must be finite and within ±{MaxGainDb} dB, got {gainDb}");
        if (!double.IsFinite(phaseDeg) || Math.Abs(phaseDeg) > MaxPhaseDeg)
            throw new ArgumentException($"I/Q phase error must be finite and within ±{MaxPhaseDeg}°, got {phaseDeg}");
        _gain = Math.Pow(10, gainDb / 20);
        double phi = phaseDeg * Math.PI / 180;
        _sin = Math.Sin(phi);
        _cos = Math.Cos(phi);
        IsIdentity = gainDb == 0 && phaseDeg == 0;
    }

    public bool IsIdentity { get; }

    public void Correct(Span<Complex> iq)
    {
        if (IsIdentity) return;
        for (int k = 0; k < iq.Length; k++)
        {
            double i = iq[k].Real;
            double qMeasured = -iq[k].Imaginary;
            double q = (_gain * qMeasured + i * _sin) / _cos;
            iq[k] = new Complex(i, -q);
        }
    }
}
