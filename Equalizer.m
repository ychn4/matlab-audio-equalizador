classdef Equalizer < matlab.apps.AppBase

    properties (Access = public)
        UIFigure matlab.ui.Figure
        EQDropDown matlab.ui.control.DropDown
        SignalDropDown matlab.ui.control.DropDown
        QEditField matlab.ui.control.NumericEditField
        Sliders matlab.ui.control.Slider
        Labels matlab.ui.control.Label
        PlayInputButton matlab.ui.control.Button
        PlayOutputButton matlab.ui.control.Button
        SaveOutputButton matlab.ui.control.Button
        ApplyEQButton matlab.ui.control.Button
        ChangeWavButton matlab.ui.control.Button
        FileNameLabel matlab.ui.control.Label
        AxFreq matlab.ui.control.UIAxes
        AxOutput matlab.ui.control.UIAxes
        PlotViewDropDown matlab.ui.control.DropDown
        Logo matlab.ui.control.Image
        Qlabel matlab.ui.control.Label
    end

    properties (Access = private)
        fs = 44100
        duration = 3
        t
        x
        y
        EQ = 0
        Q = 0.1
        loadedAudio
        fileName
        originalFs
        x_signal = 0
        bandFreqs = [100 400 1000 4000 10000]
        bandColors = {[0.8 0 0],[1 0.6 0],[1 0.9 0],[0 0.8 0],[0 0 1]}
        plotView = 0
    end

    methods (Access = public)
        function app = Equalizer
            createComponents(app);
            app.t = 0:1/app.fs:app.duration;
            app.x = sin(2*pi*1000*app.t);
            app.y = app.x;
            app.UIFigure.Visible = "on";
        end
    end

    methods (Access = private)

        % ----- EQ DropDown -----
        function EQDropDownValueChanged(app, ~)
            cla(app.AxFreq);
            cla(app.AxOutput);

            switch app.EQDropDown.Value
                case 'Parametric'
                    app.EQ = 1;
                    for i = 1:5
                        app.Labels(i).Visible = 'on';
                        app.Sliders(i).Visible = 'on';
                        app.Sliders(i).Value = 0;
                    end
                    app.QEditField.Visible = 'on';
                    app.Qlabel.Visible = 'on';

                case 'Shelving'
                    app.EQ = 2;
                    app.Qlabel.Visible = 'off';
                    app.QEditField.Visible = 'off';
                    for i = 1:5
                        app.Labels(i).Visible = 'on';
                        app.Sliders(i).Visible = 'on';
                        app.Sliders(i).Value = 0;
                    end

                otherwise
                    app.EQ = 0;
                    app.Qlabel.Visible = 'off';
                    app.QEditField.Visible = 'off';
                    for i = 1:5
                        app.Labels(i).Visible = 'off';
                        app.Sliders(i).Visible = 'off';
                    end
            end
        end

        % ----- Signal DropDown -----
        function SignalDropDownValueChanged(app, ~)
            switch app.SignalDropDown.Value
                case '1 kHz Tone'
                    app.ChangeWavButton.Visible = 'off';
                    app.x = sin(2*pi*1000*app.t);
                    app.y = app.x;
                    app.FileNameLabel.Text = '1 kHz Tone';
                    app.FileNameLabel.FontColor = [0 0.5 0];
                    app.x_signal = 0;

                case '3 dB Signal'
                    app.ChangeWavButton.Visible = 'off';
                    app.x = zeros(size(app.t));
                    app.x(1) = 10^(3/20);
                    app.y = app.x;
                    app.FileNameLabel.Text = '3 dB Signal';
                    app.FileNameLabel.FontColor = [0 0.5 0];
                    app.x_signal = 1;

                case 'Load WAV'
                    app.ChangeWavButton.Visible = 'on';
                    app.changeWavFile();
                    app.x_signal = 1;
            end

            app.applyEQ();
            app.updatePlotDisplay();
        end

        % ----- Q factor -----
        function QEditFieldValueChanged(app, event)
            app.Q = event.Value;
        end

        % ----- Apply EQ -----
        function ApplyEQButtonPushed(app, ~)
            app.applyEQ();
            app.updatePlotDisplay();
        end

        % ----- Play Input -----
        function PlayInputButtonPushed(app, ~)
            sound(app.x, app.fs);
        end

        % ----- Play Output -----
        function PlayOutputButtonPushed(app, ~)
            sound(app.y, app.fs);
        end

        % ----- Save Output -----
        function SaveOutputButtonPushed(app, ~)
            [file,path] = uiputfile("*.wav","Save Output As");
            if isequal(file,0)i
                return;
            end
            audiowrite(fullfile(path,file), app.y, app.fs);
        end

        % ----- Change WAV file -----
        function changeWavFile(app)
            cla(app.AxOutput);
            [file,path] = uigetfile("*.wav","Select WAV file");
            if isequal(file,0)
                return;
            end

            [audio, fs0] = audioread(fullfile(path,file));
            if size(audio,2) > 1
                audio = mean(audio,2);
            end

            if length(audio) > length(app.t)
                audio = audio(1:length(app.t));
            elseif length(audio) < length(app.t)
                padded = zeros(length(app.t),1);
                padded(1:length(audio)) = audio;
                audio = padded;
            end

            app.x = audio(:).';
            app.y = app.x;
            app.fs = fs0;
            app.originalFs = fs0;

            app.FileNameLabel.Text = ['Loaded: ' file];
            app.FileNameLabel.FontColor = [0 0.5 0];

            app.updatePlotDisplay();
        end

        % ----- Plot view selector -----
        function setPlotView(app)
            switch app.PlotViewDropDown.Value
                case 'f(Hz) view'
                    app.plotView = 0;
                case 't(s) view'
                    app.plotView = 1;
            end
            app.updatePlotDisplay();
        end

    end

    % ==============================================================
    %                       EQ PROCESSING
    % ==============================================================

    methods (Access = private)

        function applyEQ(app)
            if app.EQ == 1
                app.applyEQParam();
            elseif app.EQ == 2
                app.applyEQShelv();
            end
        end

        % ----- Parametric EQ -----
        function applyEQParam(app)
            gains = zeros(1,5);
            for i = 1:5
                gains(i) = app.Sliders(i).Value;
            end

            if app.Q == 0
                return;
            end

            [y_eq, Hbands] = ParametricEQ(app.x, app.fs, gains, ...
                        app.bandFreqs, app.Q);
            app.y = y_eq;

            npts = 8192;
            Htot = ones(1,npts);

            cla(app.AxFreq);

            for i = 1:5
                b = squeeze(Hbands(i,:,1));
                a = squeeze(Hbands(i,:,2));

                w = linspace(0, pi, npts);
                z = exp(1j*w);
                H = polyval(b, z.^(-1)) ./ polyval(a, z.^(-1));
                f = w * app.fs / (2*pi);

                plot(app.AxFreq, f, 20*log10(abs(H)), ...
                    'Color', app.bandColors{i}, ...
                    'LineWidth', 1, ...
                    'DisplayName', sprintf('Band %d', i));
                hold(app.AxFreq, "on");

                hL = line(app.AxFreq, ...
                    [app.bandFreqs(i) app.bandFreqs(i)], ...
                    ylim(app.AxFreq), ...
                    'Color', app.bandColors{i}, ...
                    'LineStyle','--');
                hL.Annotation.LegendInformation.IconDisplayStyle = 'off';
                Htot = Htot .* H;
            end

            plot(app.AxFreq, f, 20*log10(abs(Htot)),'k','LineWidth',2, ...
                'DisplayName','Equalizer response');
            grid(app.AxFreq,'on');
            legend(app.AxFreq, 'show');
            set(app.AxFreq,'XScale','log');
            xlim(app.AxFreq,[20 20000]);
            title(app.AxFreq,'Parametric Equalizer Response');
        end

        % ----- Shelving EQ -----
        function applyEQShelv(app)
            gains = zeros(1,5);
            for i = 1:5
                gains(i) = app.Sliders(i).Value;
            end

            [y_eq, Hbands] = ShelvingEQ(app.x, app.fs, gains, ...
                                app.bandFreqs);
            app.y = y_eq;

            npts = 8192;
            Htot = ones(1,npts);

            cla(app.AxFreq);

            for i = 1:5
                b = squeeze(Hbands(i,:,1));
                a = squeeze(Hbands(i,:,2));

                w = linspace(0, pi, npts);
                z = exp(1j*w);
                H = polyval(b, z.^(-1)) ./ polyval(a, z.^(-1));
                f = w * app.fs / (2*pi);

                plot(app.AxFreq, f, 20*log10(abs(H)), ...
                    'Color', app.bandColors{i}, ...
                    'LineWidth', 1, ...
                    'DisplayName', sprintf('Band %d', i));
                hold(app.AxFreq, "on");

                hL = line(app.AxFreq, ...
                    [app.bandFreqs(i) app.bandFreqs(i)], ...
                    ylim(app.AxFreq), ...
                    'Color', app.bandColors{i}, ...
                    'LineStyle','--');
                hL.Annotation.LegendInformation.IconDisplayStyle = 'off';
                Htot = Htot .* H;
            end

            
            plot(app.AxFreq, f, 20*log10(abs(Htot)),'k','LineWidth', 2, ...
                 'DisplayName', 'Equalizer response');
            grid(app.AxFreq,'on');
            legend(app.AxFreq, 'show');
            set(app.AxFreq,'XScale','log');
            xlim(app.AxFreq,[20 20000]);
            title(app.AxFreq,'Shelving Equalizer Response');
        end

    end

    % ==============================================================
    %                       PLOTTING FUNCTIONS
    % ==============================================================

    methods (Access = private)

        function updatePlotDisplay(app)
            if app.plotView == 0
                app.updateSignalSpectrum();
            else
                app.updateSignalResponse();
            end
        end

        % ----- Frequency domain -----
        function updateSignalSpectrum(app)
            cla(app.AxOutput);
            hold(app.AxOutput,'on');

            Nx = linspace(-app.fs/2, app.fs/2, length(app.x));

            if app.x_signal == 0
                X = fft(app.x) / length(app.t);
                Y = fft(app.y) / length(app.t);
            else
                X = fft(app.x);
                Y = fft(app.y);
            end

            X_mag = 20*log10(fftshift(abs(X)));
            
            if app.x_signal == 0
                plot(app.AxOutput, Nx, X_mag, 'b', 'LineWidth', 1.5, ...
                    'DisplayName', 'Sinusoid peak');
            else 
                plot(app.AxOutput, Nx, X_mag, 'b', 'LineWidth', 1.5, ...
                    'DisplayName', 'Input');
            end

            if ~isequal(app.x, app.y)
                Y_mag = 20*log10(fftshift(abs(Y)));
                plot(app.AxOutput, Nx, Y_mag, 'm', 'LineWidth', 1.2, ...
                    'DisplayName', 'Output');
            end
            
            if app.x_signal == 0
                ylim(app.AxOutput, [-10 inf])
            else 
                ylim(app.AxOutput, [-inf inf]);
            end

            xlim(app.AxOutput, [20 20000]);
            set(app.AxOutput,'XScale','log');
            grid(app.AxOutput,'on');
            legend(app.AxOutput, 'show');
            xlabel(app.AxOutput,'Frequency (Hz)');
            ylabel(app.AxOutput,'Magnitude (dB)');
            title(app.AxOutput,'Input-Output Spectrum');
        end

        % ----- Time domain -----
        function updateSignalResponse(app)
            cla(app.AxOutput);
            hold(app.AxOutput,'on');

            plot(app.AxOutput, app.t, app.x, 'b', 'LineWidth',1.5, ...
                'DisplayName', 'Input');

            if ~isequal(app.x, app.y)
                plot(app.AxOutput, app.t, app.y, 'm', 'LineWidth',1.2, ...
                    'DisplayName', 'Output');
            end

            xlim(app.AxOutput,[0 app.duration]);
            ylim(app.AxOutput, [-inf inf]);
            set(app.AxOutput,'XScale','linear');
            grid(app.AxOutput,'on');
            legend(app.AxOutput, 'show');
            xlabel(app.AxOutput,'Time (s)');
            ylabel(app.AxOutput,'Amplitude');
            title(app.AxOutput,'Input-Output Temporal Response');
        end
    end

    methods (Access = private)

        function createComponents(app)

            app.UIFigure = uifigure('Name','5-Band Digital Equalizer');
            app.UIFigure.Position = [300 200 900 550];

            % ----- AXES -----
            app.AxFreq = uiaxes(app.UIFigure);
            app.AxFreq.Position = [350 280 500 250];
            title(app.AxFreq,'Equalizer Frequency Response');
            xlabel(app.AxFreq,'Frequency (Hz)');
            ylabel(app.AxFreq,'Amplitude (dB)');
            grid(app.AxFreq,"on");

            app.AxOutput = uiaxes(app.UIFigure);
            app.AxOutput.Position = [350 20 500 250];
            title(app.AxOutput,'Input-Output Spectrum');
            xlabel(app.AxOutput,'Frequency (Hz)');
            ylabel(app.AxOutput,'Magnitude (dB)');
            grid(app.AxOutput,"on");

            % ----- EQ DropDown -----
            uilabel(app.UIFigure,'Text','Select Equalizer:', ...
                'Position',[70 520 120 20]);
            app.EQDropDown = uidropdown(app.UIFigure, ...
                'Items', {' ','Shelving','Parametric'}, ...
                'Position',[70 495 100 25], ...
                'ValueChangedFcn',@(src,evt)EQDropDownValueChanged(app));

            % ----- Q factor -----
            app.Qlabel = uilabel(app.UIFigure,'Text','Q factor', ...
                'Position',[175 495 60 20], 'Visible','off');
            app.QEditField = uieditfield(app.UIFigure,'numeric', ...
                'Value',0.1, 'Visible','off', ...
                'ValueChangedFcn', ...
                 @(src,evt)QEditFieldValueChanged(app,evt), ...
                'Position',[225 495 50 22]);

            % ----- Sliders -----
            for i = 1:5
                app.Labels(i) = uilabel(app.UIFigure, ...
                    'Text', sprintf('Band %d (%.0f Hz)', i, ...
                    app.bandFreqs(i)), 'Position', ...
                    [70 470-(i-1)*60 150 20], 'Visible','off');

                app.Sliders(i) = uislider(app.UIFigure, ...
                    'Limits',[-3 3], 'Value',0, ...
                    'Position',[70 460-(i-1)*60 200 3], 'Visible','off');
            end

            % ----- Signal selector -----
            uilabel(app.UIFigure,'Text','Select Signal:', ...
                'Position',[70 165 120 20]);

            app.SignalDropDown = uidropdown(app.UIFigure, ...
                'Items',{'1 kHz Tone','3 dB Signal','Load WAV'}, ...
                'Position',[70 135 120 25], ...
                'ValueChangedFcn', ...
                @(src,evt)SignalDropDownValueChanged(app));

            % ----- Buttons -----
            app.ApplyEQButton = uibutton(app.UIFigure, ...
                'Text','Apply EQ', ...
                'Position',[70 70 100 30], ...
                'ButtonPushedFcn',@(src,evt)ApplyEQButtonPushed(app));

            app.PlayInputButton = uibutton(app.UIFigure, ...
                'Text','Play Input', ...
                'Position',[175 70 100 30], ...
                'ButtonPushedFcn',@(src,evt)PlayInputButtonPushed(app));

            app.PlayOutputButton = uibutton(app.UIFigure, ...
                'Text','Play Output', ...
                'Position',[70 35 100 30], ...
                'ButtonPushedFcn',@(src,evt)PlayOutputButtonPushed(app));

            app.SaveOutputButton = uibutton(app.UIFigure, ...
                'Text','Save Output', ...
                'Position',[175 35 100 30], ...
                'ButtonPushedFcn',@(src,evt)SaveOutputButtonPushed(app));

            app.ChangeWavButton = uibutton(app.UIFigure, ...
                'Text','Change File', ...
                'Visible','off', ...
                'Position',[195 135 80 25], ...
                'ButtonPushedFcn',@(src,evt)changeWavFile(app));

            % ----- Plot view selector -----
            app.PlotViewDropDown = uidropdown(app.UIFigure, ...
                'Items',{'f(Hz) view','t(s) view'}, ...
                'Position',[755 270 90 20], ...
                'ValueChangedFcn',@(src,evt)setPlotView(app));

            % ----- Logo & File Label -----
            app.Logo = uiimage(app.UIFigure, ...
                'Position',[10 5 50 50], ...
                'ImageSource','logo.png');

            app.FileNameLabel = uilabel(app.UIFigure, ...
                'Text','1 kHz Tone', ...
                'FontColor',[0 0.5 0], ...
                'FontAngle','italic', ...
                'Position',[70 110 250 20]);
        end
    end
end