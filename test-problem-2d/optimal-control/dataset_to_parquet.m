clear;
close all;
clc;

%T = parquetread("decision_transformer_gym_replay-train.parquet");
%%
database_complete = join_db("results");
database_file = "results\database_first_it.mat";
save(database_file, "database_complete")
%%
import database.parquet_support.*
percentage_train = 0.6;
percentage_validation = 0.2;
db_list = divide_db(database_file, percentage_train, percentage_validation);
%%
file_name = "decision_transformer_satellites_rendezvous-";
main_sets = ["train", "test", "validation"];
ext = ".parquet";
for i = 1:length(db_list)
    if (~isempty(db_list{i}))
        parquetwrite(file_name + main_sets(i) + ext, db_list{i})
    end
end

%%
function database_complete = join_db(directory)
    database_complete = [];
    name_list = dir(directory);
    for i = 1:length(name_list)
        file = name_list(i);
        if file.name == "." || file.name == ".."
            continue
        end
        s  = load(directory + "/" + file.name);
        if (isfield(s,"db"))
            data = s.db;
            database_complete = [database_complete, data];
        elseif (isfield(s,"database_complete"))
            data = s.database_complete;
            database_complete = [database_complete, data];
        end
    end
end

function db_list = divide_db(database_file, percentage_train, percentage_validation)
    load(database_file, "database_complete");
    n = length(database_complete);
    feature_names = ["observations", "actions", "rewards", "dones"];
    
    
    db_train = cell2table(cell(0,4), 'VariableNames', feature_names);
    db_validation = cell2table(cell(0,4), 'VariableNames', feature_names);
    db_test = cell2table(cell(0,4), 'VariableNames', feature_names);
    for i = 1:n
        traj = database_complete(i);
        if (traj.success == 1)
            n = rand();
            if (n <= percentage_train)
                db_train = database.parquet_support.add_row(db_train, traj);
            elseif (n <= percentage_train + percentage_validation)
                db_validation = database.parquet_support.add_row(db_validation, traj);
            else
                db_test = database.parquet_support.add_row(db_test, traj);
            end
        end
    end
    db_list = {db_train, db_test, db_validation};
end