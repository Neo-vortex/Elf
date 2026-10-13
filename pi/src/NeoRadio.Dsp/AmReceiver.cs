using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>
/// AM receiver: shift the carrier to 0 Hz, decimate, filter the channel (both sidebands), then detect:
/// envelope (|x|), or synchronous (a phase-locked loop tracks the carrier and the in-phase component is
/// taken, which stays linear when selective fading weakens the carrier). The carrier's DC is removed
/// and the AGC levels the programme.
/// </summary>
public sealed class AmReceiver : IDemodulator
{
    private const double DcCornerHz = 20;
    private const double PllBandwidthHz = 30;

    private readonly Nco _shift;
    private readonly Decimator _decimator;
    private readonly ComplexFir _channel;
    private readonly bool _synchronous;
    private readonly double _dcAlpha;
    private readonly double _pllAlpha, _pllBeta;
    private readonly Agc? _agc;
    private double _dc;
    private double _phase, _freq;        // PLL: radians, radians per sample

    public AmReceiver(AmConfig config)
    {
        config.Validate();
        double half = config.BandwidthHz / 2;
        _shift = new Nco(-config.TuneOffsetHz, config.InputRate);
        _decimator = new Decimator(config.DecimationFactor, FirDesign.Lowpass(half + 300, 1200, config.InputRate));
        _channel = new ComplexFir(FirDesign.Lowpass(half + 75, 300, config.AudioRate));
        _synchronous = config.Synchronous;
        _dcAlpha = 1 - Math.Exp(-2 * Math.PI * DcCornerHz / config.AudioRate);
        // second-order loop, critically damped: natural frequency from the loop bandwidth
        double wn = 2 * Math.PI * PllBandwidthHz / config.AudioRate;
        _pllAlpha = 2 * 0.707 * wn;
        _pllBeta = wn * wn;
        _agc = config.AgcEnabled ? new Agc(config.AudioRate) : null;
    }

    public float[] Process(ReadOnlySpan<Complex> iq)
    {
        var work = iq.ToArray();
        _shift.Mix(work);
        var baseband = _decimator.Process(work);
        _channel.Process(baseband, baseband);
        var audio = new float[baseband.Length];
        for (int i = 0; i < baseband.Length; i++)
        {
            double detected;
            if (_synchronous)
            {
                var z = baseband[i] * Complex.FromPolarCoordinates(1, -_phase);
                double error = Math.Atan2(z.Imaginary, Math.Abs(z.Real) + 1e-30) * (z.Real >= 0 ? 1 : -1);
                _freq += _pllBeta * error;
                _phase += _freq + _pllAlpha * error;
                if (_phase > Math.PI) _phase -= 2 * Math.PI;
                else if (_phase < -Math.PI) _phase += 2 * Math.PI;
                detected = z.Real;
            }
            else
            {
                detected = baseband[i].Magnitude;
            }
            _dc += _dcAlpha * (detected - _dc);
            audio[i] = (float)(detected - _dc);
        }
        _agc?.Process(audio);
        return audio;
    }
}
