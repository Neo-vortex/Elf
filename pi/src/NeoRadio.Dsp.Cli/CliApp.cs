using System.Text.Json;
using NeoRadio.Dsp;

namespace NeoRadio.Dsp.Cli;

public static class CliApp
{
    public const int Ok = 0;
    public const int UsageError = 2;
    public const int InputError = 3;

    public static int Run(string[] args, TextWriter stderr)
    {
        CliOptions options;
        try
        {
            options = CliOptions.Parse(args);
        }
        catch (CliUsageException e)
        {
            stderr.WriteLine($"usage error: {e.Message}");
            stderr.WriteLine(CliOptions.Usage);
            return UsageError;
        }

        SigMfRecording recording;
        try
        {
            recording = SigMfReader.Read(options.InputPath);
        }
        catch (Exception e) when (e is InvalidDataException or IOException or JsonException or UnauthorizedAccessException)
        {
            stderr.WriteLine($"input error: {e.Message}");
            return InputError;
        }

        IDemodulator receiver;
        try
        {
            receiver = options.ToDemodulator(recording.SampleRate);
        }
        catch (ArgumentException e)
        {
            stderr.WriteLine($"usage error: {e.Message}");
            return UsageError;
        }

        new IqCorrector(options.IqGainDb, options.IqPhaseDeg).Correct(recording.Samples);
        float[] audio = receiver.Process(recording.Samples);
        try
        {
            WavWriter.WriteMono16(options.OutputPath, audio, (int)options.AudioRate);
            if (options.MetricsPath is not null)
                File.WriteAllText(options.MetricsPath, JsonSerializer.Serialize(Metrics.From(recording.Samples.Length, audio)));
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
            stderr.WriteLine($"output error: {e.Message}");
            return InputError;
        }
        return Ok;
    }
}
