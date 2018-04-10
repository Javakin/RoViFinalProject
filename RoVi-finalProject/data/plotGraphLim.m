function y = plotGraphLim(S, M, F, sL, mL, fL, FigureTitle)
    % Remove preplaced zeros
    S = S(sL:end, :);
    M = M(mL:end, :);
    F = F(fL:end, :);

    % Split up the data in positoinal and spead limits
    SQ = S(:, 1:7)';
    SU = S(:, 8:end)';
    MQ = M(:, 1:7)';
    MU = M(:, 8:end)';
    FQ = F(:, 1:7)';
    FU = F(:, 8:end)';


    % Plot the positional limits
    figure('rend','painters','pos',[150 150 1000 400])
    hold on
    %for int = 1:7
        plot((1:length(SQ))/length(SQ), max(SQ),'b')
        plot((1:length(MQ))/length(MQ), max(MQ),'r')
        plot((1:length(FQ))/length(FQ), max(FQ), 'g')
    %end

    legend('Slow Sequence', 'Medium Sequence', 'Fast Sequence')

    title([FigureTitle, ' - Displacement Exploitation'])
    xlabel('Map Completion [%]')
    ylabel('Displacement Exploitation [%]')
    hold off
    
    % Plot the Velocity limits
    figure('rend','painters','pos',[150 150 1000 400])
    hold on
    for int = 1:7
        plot((1:length(SU))/length(SU), max(SU),'b')
        plot((1:length(MU))/length(MU), max(MU),'r')
        plot((1:length(FU))/length(FU), max(FU),'g')
    end

    legend('Slow Sequence', 'Medium Sequence', 'Fast Sequence')

    title([FigureTitle, ' - Velocity Exploitation'])
    xlabel('Map Completion [%]')
    ylabel('Velocity Exploitation [%]')
    hold off


    y = 0;

end