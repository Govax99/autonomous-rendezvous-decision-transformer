function report_database_generation(batchNumber,infos,tproc,results,printLevel)
%REPORT_DATABASE_GENERATION Summary of this function goes here
%   Detailed explanation goes here
separator = "--------------------------------------------------------------------------";
if (printLevel == 0)
    % print nothing
else
    
    disp(separator)
    disp("OUTPUT REPORT FOR BATCH NUMBER " + batchNumber)
    disp(separator)
    if (printLevel == 2)
        Nsave = length(tproc);
        disp("LABEL | OBJECTIVE | TIME PROCESSOR | TIME RENDEZVOUS | SUCCESS | MESSAGE")
        for i = 1:Nsave
            disp(infos(i))
        end
    end
    
    disp("TOTAL TIME: " + floor(sum(tproc)/60) + " min")
    disp("AVERAGE CPU-TIME PER SOLUTION: " + sum(tproc)/length(tproc) + "s")
    disp("PERCENTAGE OF SUCCESS: " + sum(results == 1)/length(results)*100 + "%")
    fprintf("\n\n")
end

