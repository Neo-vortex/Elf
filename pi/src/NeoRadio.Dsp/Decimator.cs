using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>Streaming FIR decimator: computes only every <c>factor</c>-th output.</summary>
public sealed class Decimator
{
    private readonly ComplexFir _fir;
    private readonly int _factor;
    private int _phase;

    public Decimator(int factor, double[] taps)
    {
        if (factor < 1) throw new ArgumentOutOfRangeException(nameof(factor), factor, "factor must be >= 1");
        _factor = factor;
        _fir = new ComplexFir(taps);
    }

    public Complex[] Process(ReadOnlySpan<Complex> input)
    {
        var output = new List<Complex>(input.Length / _factor + 1);
        foreach (var x in input)
        {
            _fir.Push(x);
            if (_phase == 0) output.Add(_fir.Output());
            _phase = _phase + 1 == _factor ? 0 : _phase + 1;
        }
        return output.ToArray();
    }
}
