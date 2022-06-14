function [F,P] = spectre_psd_rms(x,Fs,NFFT)
if nargin<3
	NFFT = 2^10;
end
WINDOW   = hanning(NFFT);
NOVERLAP = NFFT/2;
N        =length(x)/NOVERLAP-1;		% Le nombre de moyennes
fprintf(1,'Spectre estim� avec %d moyennes\n',floor(N));
% [Pxx,F]  = psd(x,NFFT,Fs,WINDOW,NOVERLAP);
[Pxx,F]  = pwelch(x,WINDOW,NOVERLAP,NFFT,Fs);

WINDOW_COR = norm(WINDOW)^2/sum(WINDOW)^2;
XX_pic   = 2*sqrt(Pxx*WINDOW_COR);
XX_rms   = XX_pic/sqrt(2);
P        = 20*log10(XX_rms);