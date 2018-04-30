function [ab] = load_learning_data(path1, plotTrainingData)

    load ([path1 '/Data/demo_human']);
    [atr, atr_mean] = fill_with_velocity(human_demo_data);  % training data for Baxter
    clear human_demo_data
     
     load ([path1 '/Data/demo_baxter']);
     [btr, btr_mean]    = fill_with_velocity(baxter_demo_data);  % training data for Human
     clear baxter_demo_data
   
    % training data
    for k=1:length(atr.q)
        abtr.q{k,:} = [atr.q{k}(:,1:6)  btr.q{k}];
    end
    
    abtr_mean   = [atr_mean(:,1:6)    btr_mean];

    ab.tr = abtr;      
    ab.tr_mean   = abtr_mean;

    %% Plot stuff of interest
    if ~isempty(plotTrainingData)

        plot_dataVSsample('A', atr, ats, plotTrainingData.test, plotTrainingData.test_savePlot)
        if plotTrainingData.xy
            plot_xy_letter('A', atr, atr_mean, plotTrainingData.xy_savePlot);
        end        

        plot_dataVSsample('B', btr, bts, plotTrainingData.test, plotTrainingData.test_savePlot)
        if plotTrainingData.xy
            plot_xy_letter('B', btr, btr_mean, plotTrainingData.xy_savePlot);
        end                          
    end
    
    
end


function [newltr, mean_] = fill_with_velocity(ltr)
% adding velocity as NaN values to keep the same format as previous code.

    for k=1:length(ltr)
        [nx, ny] = size(ltr{k});
        putVel = zeros(nx, 2*ny);
        for l=1:ny
            putVel(:,(2*l)-1) =  ltr{k}(:,l);
            putVel(:,2*l) = 0.*ltr{k}(:,l)./0;
        end
        tmp = putVel;
        
        % forcing sampling to 100 points
        xorig     = linspace(1, 100, size(tmp,1));
        xresample = linspace(1, 100);
        tmp2      = interp1(xorig, tmp, xresample);
        
        newltr.q{k,:} = tmp2;
    end
    
    me = zeros(size(newltr.q{1}));
    for k=1:length(newltr.q)
       me = me+ newltr.q{k};
    end
    mean_ = me./length(ltr);
    
    if 0
        figurew;
        for k=1:length(ltr)
            plot(newltr.q{k}(:,1), newltr.q{k}(:,3), SGRAY(0.5));
            plot(mean_(:,1), mean_(:,3), SRED(2));
        end
    end 
end

function [] = plot_dataVSsample(letterName, data, dataTest, plotTestData, saveplot)

    h = figurew([letterName ' training']);
    plotTrajectoryStatistics(data, [1:1:length(data.q{1})], 'b' , [1]);  
    plotTrajectoryStatistics(data, [1:1:length(data.q{1})], 'r' , [3]);
    title([letterName '. blue:X, red:Y gray: test']);
    if plotTestData
        for k=1:length(dataTest.q)
            plot(dataTest.q{k}, SGRAY(0.5) );
        end  
    end
    ylim([-0.2    1.2]);
    
    if saveplot
        plot2svg([letterName '_joint.svg'], h, 'png');
    end
    
end

function [] = plot_xy_letter(letterName, data, data_mean, saveplot)

    h = figurew([letterName ' letter']);
    for k=1:length(data.q)
        plot(data.q{k}(:,1), data.q{k}(:,3), SGRAY(2, 0.8));
    end  
    plot(data_mean(:,1), data_mean(:,3), SBLUE(2));
    axis([-0.2   1   -0.2  1]);
    
    if saveplot
        plot2svg([letterName '_cart.svg'], h, 'png');
    end
    
end



