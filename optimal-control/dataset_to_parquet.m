clear;
close all;
clc;

%T = parquetread("decision_transformer_gym_replay-train.parquet");
%%
import database.parquet_support.*
load database_v1.mat
feature_names = ["observations", "actions", "rewards", "dones"];


db_train = cell2table(cell(0,4), 'VariableNames', feature_names);
db_validation = cell2table(cell(0,4), 'VariableNames', feature_names);
db_test = cell2table(cell(0,4), 'VariableNames', feature_names);
p_train = 1;
p_validation = 0;
for i = 1:length(db)
    traj = db(i);
    if (traj.success == 1)
        n = rand();
        if (n <= p_train)
            db_train = database.parquet_support.add_row(db_train, traj);
        elseif (n <= p_train + p_validation)
            db_validation = database.parquet_support.add_row(db_validation, traj);
        else
            db_test = database.parquet_support.add_row(db_test, traj);
        end
    end
end
db_list = {db_train, db_test, db_validation};
%%
file_name = "decision_transformer_satellites_rendezvous-";
main_sets = ["train", "test", "validation"];
ext = ".parquet";
for i = 1:length(db_list)
    if (~isempty(db_list{i}))
        parquetwrite(file_name + main_sets(i) + ext, db_list{i})
    end
end