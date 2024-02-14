Analysis of signal in time 
figure(1)
segnale = data.segnale
%fs = data.Fs
tempo = data.time

plot(tempo,segnale)
xlim([0 5])
xlabel('Tempo[s]')
ylabel('segnale')


% band pass filter 
fs = 360;
emg_signal = segnale;
t = 0:0.001:1;
% Define filter parameters
low_freq = 80; % Low cutoff frequency in Hz
high_freq = 120; % High cutoff frequency in Hz
order = 4; % Filter order (you can adjust this)

% Design a band-pass Butterworth filter
[b, a] = butter(order, [low_freq, high_freq] / (fs/ 2));

% Apply the band-pass filter to the EMG signal
filtered_emg = filtfilt(b, a, emg_signal);

% Plot the original and filtered EMG signals
figure;
subplot(2,1,1);
plot(tempo, emg_signal);
xlim([0 5])
title('Original EMG Signal');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(2,1,2);
plot(tempo, filtered_emg);
xlim([0 5])
title('Band-Pass Filtered EMG Signal');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;


%fully wave rectification
% Sample EMG signal 


emg_signal = segnale;

% Full-wave rectification
rectified_emg = abs(emg_signal);

% Plot the original and rectified signals
figure;
subplot(2,1,1);
plot(tempo,emg_signal);
xlim([0 5])
title('Original EMG Signal');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(2,1,2);
plot(tempo, rectified_emg);
xlim([0 5])
title('Rectified EMG Signal');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

Analisi del comportamento in frequenza 
figure(2)
u1 = (0:length(segnale)-1)/length(segnale) 
plot(u1,(abs(fft(segnale))))
xlim([0,0.3])
ylim([0 4000])
title('Trasformata di Fourier del segnale')
xlabel('Frequenza[Hz]')
ylabel('FFT')

%poiche l'andamento del segnale nel tempo e nella frequenza 
%è minore rispetto al rumore, è opportuno applicare un filtro fir

Filtro fir
M = 100 % M numero di vincoli di progettazione
N = 100 % N numero di gradi di libertà 
u = linspace(0,0.5,M)'; %vettore con freuqneze normalizzate in cui sono appliacti i vincoli definiti su H 
H = (u < 0.071) 
u = repmat(u,1,N-1) %ripete u per una riga e N-1 colonne
n = 1:N-1
n = repmat(n,M,1)
C = [ones(M,1) 2*cos(2*pi*u.*n)] %ones:crea una matrice Mx1 di tutti 1
h = C\H
h = [h(end:-1:2);h]

 

figure(4)
plot(tempo,segnale,'Color','r')
hold on
plot(tempo,conv(segnale,h,'same'),'k','LineWidth',2)
xlim([0 3])
xlabel('Tempo[s]')
ylabel('Ampiezza')
hold off 


Ottimizzazione rispetto alla frequenza di taglio (SNR)
f = [0.02:0.001:0.25]
frequenza = length(f)
SignalPower = @(x) norm(x(:)-mean(x(:)))^2/length(x(:)) 
for i = 1:frequenza
    M = 100  %vincoli
    N = 100 %parametri liberi
    u = linspace(0,0.5,M)'
    H = (u < f(i))
    u = repmat(u,1,N-1)
    n = 1:N-1
    n = repmat(n,M,1)
    C = [ones(M,1) 2*cos(2*pi*u.*n)]
    h = C\H
    h = [h(end:-1:2);h]
    segfiltrato = conv(segnale,h,'same')
    segmento_segnale = segfiltrato((tempo>0.805)&(tempo<1.330))
    segmento_rumore = segfiltrato((tempo>1.330)&(tempo<1.891))
    SNR(i) = [10*log10(SignalPower(segmento_segnale)/SignalPower(segmento_rumore))]
    w(i) = f(i)
end

[SNR_OPT,w_opt] = max(SNR)
ftaglio_opt = f(w_opt)

disp(['frequenza ottimizzata=' num2str(ftaglio_opt)])

disp(['SNR relativo=' num2str(SNR_OPT)])

figure
plot(f,SNR)

