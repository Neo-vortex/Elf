namespace NeoRadio.Dsp;

/// <summary>AM receiver settings. TuneOffsetHz is the carrier frequency minus LO; BandwidthHz is the
/// channel width (both sidebands); Synchronous selects carrier-tracking detection.</summary>
public sealed record AmConfig(
    double InputRate,
    double AudioRate,
    double TuneOffsetHz,
    double BandwidthHz = 9000,
    bool Synchronous = false,
    bool AgcEnabled = true)
{
    public int DecimationFactor => (int)Math.Round(InputRate / AudioRate);

    public void Validate()
    {
        if (InputRate <= 0 || AudioRate <= 0)
            throw new ArgumentException("sample rates must be positive");
        double ratio = InputRate / AudioRate;
        if (ratio < 1 || Math.Abs(ratio - Math.Round(ratio)) > 1e-9)
            throw new ArgumentException($"input rate {InputRate} must be an integer multiple of audio rate {AudioRate}");
        double half = BandwidthHz / 2;
        if (BandwidthHz <= 0 || half >= AudioRate / 2 - 300)
            throw new ArgumentException($"AM bandwidth {BandwidthHz} Hz must be below {AudioRate - 600} Hz at audio rate {AudioRate}");
        double limit = ReceiverConfig.UsableFraction * InputRate;
        if (Math.Abs(TuneOffsetHz) + half > limit)
            throw new ArgumentException(
                $"carrier offset {TuneOffsetHz} Hz with bandwidth {BandwidthHz} Hz is outside the usable window ±{limit} Hz");
    }
}
