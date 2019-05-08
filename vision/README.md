tensor.pt is pytorch 2 layer neural network which takes an 69X49 image normalizes it by dividing by 255 and predicts wheather it is a patch or lane.

training_neural_net.ipyb is a jupyter notebook for training the neural net.

grid_lane is a primitive object removal and lane detection code which takes the 2b-g channel and divides it into grids and provides various thresholds for removings objects with relatively low computational costs.
