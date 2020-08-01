## Build from source

mkdir build && cd build. 
cmake ..  
make. 

## Create the path graph from the track
ln -s ../assets/config.yaml .  
./trackBuilder -i ../assets/track_test.png  

## Start the simulation
./minibots. 
