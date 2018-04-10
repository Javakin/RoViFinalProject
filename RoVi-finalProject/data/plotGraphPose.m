function y = plotGraphPose(S, M, F, sL, mL, fL, FigureTitle)
    % Remove preplaced zeros
    S = S(sL:end, :);
    M = M(mL:end, :);
    F = F(fL:end, :);

    % Split up the data in Q vectors and U vectors
    SQ = S(:, 1:7)';
    SU = S(:, 8:end)';
    MQ = M(:, 1:7)';
    MU = M(:, 8:end)';
    FQ = F(:, 1:7)';
    FU = F(:, 8:end)';


    % Plot the Statevector
    figure('rend','painters','pos',[150 150 1000 400])
    hold on
    for int = 1:7
        plot((1:length(SQ))/length(SQ), SQ(int, :),'b')
        plot((1:length(MQ))/length(MQ), MQ(int, :),'r')
        plot((1:length(FQ))/length(FQ),FQ(int, :), 'g')
    end

    legend('Slow Sequence', 'Medium Sequence', 'Fast Sequence')

    title([FigureTitle, ' Joint Configuration'])
    xlabel('Map Completion [%]')
    ylabel('Joint Configuration [rad]')
    hold off

    % plot dU xyz
    figure('rend','painters','pos',[150 150 1000 400])
    hold on
    for int = 1:3
        plot((1:length(SU))/length(SU), SU(int, :),'b')
        plot((1:length(MU))/length(MU), MU(int, :),'r')
        plot((1:length(FU))/length(FU), FU(int, :),'g')
    end

    legend('Slow Sequence', 'Medium Sequence', 'Fast Sequence')

    title([FigureTitle, ' Tool Pose (X,Y,Z)'])
    xlabel('Map Completion [%]')
    ylabel('Tool Displacement [Meters]')
    hold off

    % plot dU eaa
    figure('rend','painters','pos',[150 150 1000 400])
    hold on
    for int = 4:6
        plot((1:length(SU))/length(SU), SU(int, :),'b')
        plot((1:length(MU))/length(MU), MU(int, :),'r')
        plot((1:length(FU))/length(FU), FU(int, :),'g')
    end

    legend('Slow Sequence', 'Medium Sequence', 'Fast Sequence')

    title([FigureTitle, ' Tool EAA (\thetaX,\thetaY,\thetaZ)'])
    xlabel('Map Completion [%]')
    ylabel('Tool Rotational Displacement [rad]')
    hold off

    y = 0;

end