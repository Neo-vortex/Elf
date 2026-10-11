using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>Streaming FIR filter with real taps and complex samples.</summary>
public sealed class ComplexFir
{
    private readonly double[] _taps;
    private readonly Complex[] _history;
    private int _pos;

    public ComplexFir(double[] taps)
    {
        if (taps.Length == 0) throw new ArgumentException("taps must not be empty", nameof(taps));
        _taps = taps;
        _history = new Complex[taps.Length];
    }

    public void Push(Complex x)
    {
        _pos = _pos + 1 == _history.Length ? 0 : _pos + 1;
        _history[_pos] = x;
    }

    public Complex Output()
    {
        Complex acc = Complex.Zero;
        int idx = _pos;
        for (int i = 0; i < _taps.Length; i++)
        {
            acc += _history[idx] * _taps[i];
            idx = idx == 0 ? _history.Length - 1 : idx - 1;
        }
        return acc;
    }

    public Complex Step(Complex x)
    {
        Push(x);
        return Output();
    }

    public void Process(ReadOnlySpan<Complex> input, Span<Complex> output)
    {
        if (output.Length < input.Length) throw new ArgumentException("output too short", nameof(output));
        for (int i = 0; i < input.Length; i++) output[i] = Step(input[i]);
    }
}
