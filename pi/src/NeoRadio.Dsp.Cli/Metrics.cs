using System.Text.Json.Serialization;

namespace NeoRadio.Dsp.Cli;

public sealed record Metrics(
    [property: JsonPropertyName("input_samples")] int InputSamples,
    [property: JsonPropertyName("audio_samples")] int AudioSamples,
    [property: JsonPropertyName("audio_rms")] double AudioRms,
    [property: JsonPropertyName("audio_peak")] double AudioPeak)
{
    public static Metrics From(int inputSamples, float[] audio)
    {
        double sumSq = 0, peak = 0;
        foreach (float s in audio)
        {
            sumSq += (double)s * s;
            peak = Math.Max(peak, Math.Abs(s));
        }
        return new Metrics(inputSamples, audio.Length, audio.Length == 0 ? 0 : Math.Sqrt(sumSq / audio.Length), peak);
    }
}
