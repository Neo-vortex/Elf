namespace NeoRadio.Dsp;

/// <summary>FIR filter design helpers.</summary>
public static class FirDesign
{
    /// <summary>
    /// Blackman-windowed sinc lowpass. <paramref name="cutoffHz"/> is the −6 dB point; the passband is
    /// flat to cutoff − transition/2 and the stopband (≈ −74 dB) starts at cutoff + transition/2.
    /// </summary>
    public static double[] Lowpass(double cutoffHz, double transitionHz, double sampleRate)
    {
        if (cutoffHz <= 0 || cutoffHz >= sampleRate / 2)
            throw new ArgumentOutOfRangeException(nameof(cutoffHz), cutoffHz, "cutoff must be between 0 and fs/2");
        if (transitionHz <= 0)
            throw new ArgumentOutOfRangeException(nameof(transitionHz), transitionHz, "transition must be positive");

        int n = (int)Math.Ceiling(5.5 * sampleRate / transitionHz);
        if (n % 2 == 0) n++;
        int m = n - 1;
        double fc = cutoffHz / sampleRate;
        var taps = new double[n];
        double sum = 0;
        for (int i = 0; i < n; i++)
        {
            double k = i - m / 2.0;
            double sinc = k == 0 ? 2 * fc : Math.Sin(2 * Math.PI * fc * k) / (Math.PI * k);
            double window = 0.42 - 0.5 * Math.Cos(2 * Math.PI * i / m) + 0.08 * Math.Cos(4 * Math.PI * i / m);
            taps[i] = sinc * window;
            sum += taps[i];
        }
        for (int i = 0; i < n; i++) taps[i] /= sum;
        return taps;
    }
}
