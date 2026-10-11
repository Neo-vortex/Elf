using System.Numerics;

namespace NeoRadio.Dsp;

/// <summary>
/// Single-sideband receiver: shift the wanted passband centre to 0 Hz, decimate, apply the channel
/// filter, shift back to the audio centre and take the real part.
/// </summary>
public sealed class Receiver
{
    private readonly Nco _shiftIn;
    private readonly Decimator _decimator;
    private readonly ComplexFir _channel;
    private readonly Nco _shiftOut;
    private readonly Agc? _agc;

    public Receiver(ReceiverConfig config)
    {
        config.Validate();
        double sign = config.Sideband == Sideband.Upper ? 1 : -1;
        double centre = (config.LowCutHz + config.HighCutHz) / 2;
        double half = (config.HighCutHz - config.LowCutHz) / 2;

        _shiftIn = new Nco(-(config.TuneOffsetHz + sign * centre), config.InputRate);
        _decimator = new Decimator(
            config.DecimationFactor,
            FirDesign.Lowpass(0.4 * config.AudioRate, 0.2 * config.AudioRate, config.InputRate));
        _channel = new ComplexFir(FirDesign.Lowpass(half + 75, 150, config.AudioRate));
        _shiftOut = new Nco(sign * centre, config.AudioRate);
        _agc = config.AgcEnabled ? new Agc(config.AudioRate) : null;
    }

    public float[] Process(ReadOnlySpan<Complex> iq)
    {
        var work = iq.ToArray();
        _shiftIn.Mix(work);
        var baseband = _decimator.Process(work);
        _channel.Process(baseband, baseband);
        _shiftOut.Mix(baseband);
        var audio = new float[baseband.Length];
        for (int i = 0; i < audio.Length; i++) audio[i] = (float)baseband[i].Real;
        _agc?.Process(audio);
        return audio;
    }
}
