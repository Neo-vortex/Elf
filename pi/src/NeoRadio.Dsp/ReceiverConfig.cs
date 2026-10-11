namespace NeoRadio.Dsp;

public enum Sideband { Upper, Lower }

/// <summary>Receiver settings. TuneOffsetHz is dial frequency minus LO.</summary>
public sealed record ReceiverConfig(
    Sideband Sideband,
    double InputRate,
    double AudioRate,
    double TuneOffsetHz,
    double LowCutHz,
    double HighCutHz,
    bool AgcEnabled = true)
{
    public const double UsableFraction = 0.45;

    public int DecimationFactor => (int)Math.Round(InputRate / AudioRate);

    public void Validate()
    {
        if (InputRate <= 0 || AudioRate <= 0)
            throw new ArgumentException("sample rates must be positive");
        double ratio = InputRate / AudioRate;
        if (ratio < 1 || Math.Abs(ratio - Math.Round(ratio)) > 1e-9)
            throw new ArgumentException($"input rate {InputRate} must be an integer multiple of audio rate {AudioRate}");
        if (LowCutHz < 0 || LowCutHz >= HighCutHz)
            throw new ArgumentException($"audio passband [{LowCutHz}, {HighCutHz}] Hz is invalid");
        if (HighCutHz >= AudioRate / 2 - 100)
            throw new ArgumentException($"high cut {HighCutHz} Hz must be below {AudioRate / 2 - 100} Hz");
        double limit = UsableFraction * InputRate;
        if (Math.Abs(TuneOffsetHz) + HighCutHz > limit)
            throw new ArgumentException(
                $"tune offset {TuneOffsetHz} Hz with high cut {HighCutHz} Hz is outside the usable window ±{limit} Hz");
    }
}
