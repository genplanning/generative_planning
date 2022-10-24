# Generative Planning-inpainting model

Pytorch implementation of our proposed inpaining model for lidar image to semantic map translation

## Prerequisites
- Linux or macOS
- Python 2 or 3
- NVIDIA GPU (11G memory or larger) + CUDA cuDNN

## Getting Started
### Installation
- Install PyTorch and dependencies from http://pytorch.org
- Install python libraries [dominate](https://github.com/Knio/dominate).
```bash
pip install dominate
```
## Preprocess dataset
If you download our released original dataset without processing, then you need to preprocess our dataset first, by combining every corresponding input and groundtruth image into one image by running the following command:
```bash
python datasets/combine_A_and_B.py --fold_A INPUT_IMAGE_DATA_PATH --fold_B GROUNDTRUTH_DATA_PATH --fold_AB The folder you want to put your combined images in
```


### Testing
- A few example test images from our dataset are included in the `datasets/test` folder (Note: for testing convenience, we only upload processed paired images verison here , so the images in this folder are the combined input (left) and groudtruth (right) images.
- Please download our pre-trained model from [here](https://drive.google.com/file/d/1ugicQHOmuNJnytlLF-NRryJxU5efkdpJ/view?usp=sharing) (google drive link), and put it under `./checkpoints/fixed_render_hd1/`
- Test the model:
```bash
#!./scripts/test_256p.sh
python test.py --dataroot datasets/test --name fixed_render_hd1  --model pix2pix
```
The test results will be saved to a html file here: `./results/fixed_render_hd1/test_latest/index.html`.

More example scripts can be found in the `scripts` directory.


### Dataset
- We use the our processed version of KITTI-360 dataset ([official website](http://www.cvlibs.net/datasets/kitti-360/)). We will release our processed dataset for training after paper gets accepted.


### Training
- Train a model at 1024 x 512 resolution:
```bash
#!./scripts/train_256p.sh
python train.py --dataroot DATASET_PATH  --model pix2pix --name fixed_render_hd1
```
- To view training results, please checkout intermediate results in `./checkpoints/fixed_render_hd1/web/index.html`.
If you have tensorflow installed, you can see tensorboard logs in `./checkpoints/fixed_render_hd1/logs` by adding `--tf_log` to the training scripts.


## More Training/Test Details
- Flags: see `options/train_options.py` and `options/base_options.py` for all the training flags; see `options/test_options.py` and `options/base_options.py` for all the test flags.


## Acknowledgments
This code and readme instructions borrows heavily from [pix2pixHD](https://github.com/NVIDIA/pix2pixHD) and [Pix2Pix](https://github.com/junyanz/pytorch-CycleGAN-and-pix2pix).
