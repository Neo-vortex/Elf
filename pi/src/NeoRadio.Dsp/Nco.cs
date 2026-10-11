using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>Phase-continuous complex frequency shifter.</summary>
public sealed class Nco
{
    private readonly Complex _step;
    private Complex _phasor = Complex.One;
    private int _sinceNormalize;

    public Nco(double frequencyHz, double sampleRate)
    {
        double w = 2 * Math.PI * frequencyHz / sampleRate;
        _step = new Complex(Math.Cos(w), Math.Sin(w));
    }

    public void Mix(Span<Complex> data)
    {
        for (int i = 0; i < data.Length; i++)
        {
            data[i] *= _phasor;
            _phasor *= _step;
            if (++_sinceNormalize == 1024)
            {
                _phasor /= _phasor.Magnitude;
                _sinceNormalize = 0;
            }
        }
    }
}
