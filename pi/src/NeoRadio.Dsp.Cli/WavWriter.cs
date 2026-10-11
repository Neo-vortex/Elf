using System.Text;

namespace NeoRadio.Dsp.Cli;

public static class WavWriter
{
    /// <summary>Writes 16-bit PCM mono; samples are clipped to [-1, 1].</summary>
    public static void WriteMono16(string path, ReadOnlySpan<float> samples, int sampleRate)
    {
        using var w = new BinaryWriter(File.Create(path), Encoding.ASCII);
        int dataBytes = samples.Length * 2;
        w.Write(Encoding.ASCII.GetBytes("RIFF"));
        w.Write(36 + dataBytes);
        w.Write(Encoding.ASCII.GetBytes("WAVE"));
        w.Write(Encoding.ASCII.GetBytes("fmt "));
        w.Write(16);
        w.Write((short)1);              // PCM
        w.Write((short)1);              // mono
        w.Write(sampleRate);
        w.Write(sampleRate * 2);        // byte rate
        w.Write((short)2);              // block align
        w.Write((short)16);             // bits per sample
        w.Write(Encoding.ASCII.GetBytes("data"));
        w.Write(dataBytes);
        foreach (float s in samples)
            w.Write((short)Math.Round(Math.Clamp(s, -1f, 1f) * 32767));
    }
}
