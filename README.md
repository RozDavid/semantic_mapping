# 3D Semantic Label Transfer in Human-Robot Collaboration
### Supplementary material for our ICCV 2021 CVinHRC workshop - oral presentation

**Abstract -** We  tackle  two  practical  problems  in  robotic  scene  un-derstanding.  First, the computational requirements of cur-rent semantic segmentation algorithms are prohibitive fortypical robots. Second, the viewpoints of ground robots arequite different from the typical human viewpoints of trainingdatasets which may lead to misclassified objects from robotviewpoints. We present a system for sharing and reusing 3Dsemantic information between multiple agents with differentviewpoints. We first co-localize all agents in the same coor-dinate system.  Next, we create a 3D dense semantic modelof the space from human viewpoints close to real time.  Fi-nally, by re-rendering the model’s semantic labels (and/ordepth maps) from the ground robots’ own estimated view-points and sharing them over the network, we can give 3Dsemantic understanding to simpler agents. We evaluate thereconstruction quality and show how tiny robots can reuseknowledge about the space collected by more capable peers.

[ICCV Open Access](https://openaccess.thecvf.com/content/ICCV2021W/CVinHRC/papers/Rozenberszki_3D_Semantic_Label_Transfer_in_Human-Robot_Collaboration_ICCVW_2021_paper.pdf) | [Videos](https://youtube.com/playlist?list=PLkDXKdUTX9GPjOI4yg7y5xhLEO53JQhze)

Supporting code for the project semantic 3D reconstruction in simulated and real environments and remote semantic label transfer.

- The majority of the pipeline can be found under the [kimera_ws](https://github.com/RozDavid/semantic_mapping/tree/master/kimera_ws) directory
- Supporting scripts in the [scripts](https://github.com/RozDavid/semantic_mapping/tree/master/scripts) folder
- The simulation environment in [habitat](https://github.com/RozDavid/semantic_mapping/tree/master/habitat) 
- Finally semantic segmentation related nodes are in the [semantic_segmentation](https://github.com/RozDavid/semantic_mapping/tree/master/semantic_segmentation) folder.

Additional README-s and installation/execution descriptions can be found in the corresponding folders. 
If you have problems with the installation or have questions regarding the implemnetation/paper please open an issue. 

If you find our work useful please cite with: 

```text
@inproceedings{3d_label_transfer,
  title={3D Semantic Label Transfer in Human-Robot Collaboration},
  author={Rozenberszki, David and Soros, Gabor and Szeier, Szilvia and Lorincz, Andras},
  booktitle={Proceedings of the IEEE/CVF International Conference on Computer Vision},
  pages={2602--2611},
  year={2021}
}
```  

