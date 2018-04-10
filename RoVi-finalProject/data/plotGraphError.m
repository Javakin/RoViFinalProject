function y = plotGraphError(S, M, F, sL, mL, fL, FigureTitle)
    % remove preplaced zeros
    S = S(sL:end);
    M = M(mL:end);
    F = F(fL:end);
    
    % plot the graph
    figure('rend','painters','pos',[150 150 1000 400])
    %figure()
    hold on
    plot((1:length(S))/length(S), S)
    plot((1:length(M))/length(M), M)
    plot((1:length(F))/length(F),F)
   
    legend('Slow Sequence', 'Medium Sequence', 'Fast Sequence')
    title(FigureTitle)
    xlabel('Map Completion [%]')
    ylabel('Eucliedian Pixel Error [Pixel]')
    
    hold off
    y = 0;
end