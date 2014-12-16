raw = csvread('jitter_pos.csv',1);
raw_far = csvread('jitter_pos_far.csv',1);
p = raw(:,4:6);
p_far = raw_far(:,4:6);