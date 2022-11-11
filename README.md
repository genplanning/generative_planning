# Generative_planning
the GitHub repo for CORL 2022 accepted Oral paper "Planning Paths through Occlusions in Urban Environments"

# Inpainting model
To generate the inpainted semantic map, first type the command

```bash
cd inpainting/
```
The code and instructions to run the inpainting model are under the 'inpainting' folder.

# path planning
Then take the generated semantic maps from the inpainting foder and then type the command

```bash
cd ../
cd planner
```
The code and instructions to run the path planning method are under the 'planner' folder.

# Note for test the model
To test our model, please run the inference instruction under the 'inpainting' folder first. Then use the predicted semantic map to run the planning stage.

# Results
Here are some planning results by using our generative planning pipeline. 

Explanation for the demonstrated images: (top) the semantic
point cloud; the white dot is vehicle pose and the green dot is the end point. 
(Middle) The skeletonized road with waypoints (blue dots), (bottom) the planned path; the start is
in pink, the goal is in green, and the planned nodes are in blue

Figure 1:
<img src='results/10401.png' align="center" width=1000>
Figure 2:
<img src='results/10425.png' align="center" width=1000>
Figure 3:
<img src='results/11019.png' align="center" width=1000>
