namespace NeoRadio.Dsp;

/// <summary>
/// Automatic gain control: instant-attack peak follower with exponential decay. Because the envelope is
/// never below the current sample, the output magnitude never exceeds the target (no start-up overshoot).
/// </summary>
public sealed class Agc
{
    private readonly double _decay;
    private readonly double _target;
    private double _envelope;

    public Agc(double sampleRate, double decaySeconds = 0.5, double target = 0.3)
    {
        _decay = 1 - Math.Exp(-1 / (decaySeconds * sampleRate));
        _target = target;
    }

    public void Process(Span<float> audio)
    {
        for (int i = 0; i < audio.Length; i++)
        {
            double m = Math.Abs(audio[i]);
            if (m > _envelope) _envelope = m;
            else _envelope += _decay * (m - _envelope);
            audio[i] = _envelope > 1e-12 ? (float)(audio[i] * _target / _envelope) : 0f;
        }
    }
}
