using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>A demodulator: I/Q at the input rate in, audio at the audio rate out (stateful across calls).</summary>
public interface IDemodulator
{
    float[] Process(ReadOnlySpan<Complex> iq);
}
