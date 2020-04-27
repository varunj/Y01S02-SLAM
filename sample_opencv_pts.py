import numpy as np
import pandas as pd

pts_file = "./2014-12-12-10-45-15_pts_history_file_my.txt"
new_pts_file = "./pts_history_file.txt"
id_file = "./incoming_id_file.txt"
ids = np.loadtxt("./incoming_id_file.txt",dtype=int)
df = pd.DataFrame(columns =["id","x","y","z","color"] )
chunksize = 10 ** 7
count = 0
for chunk in pd.read_csv(pts_file,sep = " ",chunksize=chunksize,names=["id","x","y","z","color"]):
    chunk = chunk.astype({"id":"int"})
    chunk_satisfy_condition = chunk[chunk["id"].isin(ids)] 
    sample_num = int((chunk_satisfy_condition.shape[0]) * (215/320000.0))
    print(sample_num)
    chunk_satisfy_condition = chunk_satisfy_condition.sample(n =sample_num )
    print(chunk_satisfy_condition.shape)
    if chunk_satisfy_condition.shape[0] == 0:
        continue
    df = pd.concat([df,chunk_satisfy_condition], ignore_index = True)
    count +=1
    print(count)

df = df.sort_values(by=["id"])
df.to_csv(new_pts_file,index = False,sep = " ", header= None)

    


