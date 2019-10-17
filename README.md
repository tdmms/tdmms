## Two-dimensional materials manufacturing system (2DMMS)
Open-source software for automated searching and assembly for van der Waals heterostructures

*Note that this webpage is currently under development.*

### About 2DMMS
Open-source platform for aunotnomous robotic searching and assembly of van der Waals heterostructures. The softwre is dedicated to minimize the human intervention involved in fabrication of van der Waals heterostructures. Using the software in combination with the hardware provides the following functionalities:

* Automated searching for atomically thin exfoliated 2D crystals on SiO2/Si substrate
* Building the database of opitcal microscope images of detected 2D crystals
* Computer assisted design of van der Waals heterostructures by selecting the 2D crystals from the database
* Semi-automated assembly of designed van der Waals heterostructures

### License
* The software is licenced under BSD-3-Clause License

### Software Requirements
The following software components are required.
* Ubuntu 14.04 or higher
* ROS Indigo
* QT 4.0.0 or higher
* HALCON12 Image processing library
* Open CL
* MySQL
* Tensorflow
* Keras

### Hardware support
The following hardware comopnents are supported.

#### Automated Optical Microscope
|Unit| Component | Parts ID| Manufacturer|
|:-----------:|:------------:|:------------:|:------------:|
| Autofocus Microscope | Autofocus System | AF-77VB-N60LP-TDS                 | Chuo Precision |
|                      |                  | AF-77VB                           | Chuo Precision |
|                      | CMOS Camera      | Basler ACE acA2000-340kc          | Basler         |
|                      |                  | AF Pattern Positioner             | Chuo Precision |
|                      |                  | AFC-5 Autofocus Controller        | Chuo Precision |
|                      |                  | AF-61ZA Movement module           | Chuo Precision |
|                      |                  | Motor Cables 3m for each          | Chuo Precision |
|                      |                  |Optical Microsocpe Pole (Original) | Designed in-house|
| Illumination Unit    | Autofocus part   |TKG-3780-500 AF Illumination Source| Hayashi Watch Works|
|                      |                  | Red (780nm)                       | Hayashi Watch Works|
|                      |                  |LP-0805RA LED Power Module         | Hayashi Watch Works|
|                      |                  |TKG-3790-400 Robot Cable 3m        | Hayashi Watch Works|
|                      | Imaging          |LA-150FBU Halogen Lamp             | Hayashi Watch Works|
|                      |                  |FB-LGB1-5L1000 Light Guide 1m      | Hayashi Watch Works|
|                      |                  |NIC-100 LAN Control Unit           | Hayashi Watch Works|
|                      |                  |NC-10E Connection Cable            | Hayashi Watch Works|
|                      |                  |LAN Cable 3m                       | Hayashi Watch Works|


Halogen Lamp     | LA-150FBU | Chuo Precision/Hayashi Watch Works|
|                      | LAN Control Unit | NIC-100   | Chuo Precision/Hayashi Watch Works|

## Install dependencies for Ubuntu 14.04
`% sudo apt-get install ros-indigo-desktop-full ros-indigo-nmea-msgs ros-indigo-nmea-navsat-driver ros-indigo-sound-play`

## Changes
* **beta-1** initial public release.
* **beta-2** support for deep learning inference

## Citations
The detailed descriptions of this system are provided in the paper;
* "Autonomous robotic searching and assembly of two-dimensional crystals to build van der Waals superlattices", Satoru Masubuchi *et al.*, Nature Communications (2018). 
* "Classifying optical microscope images of exfoliated graphene flakes by data-driven machine learning", Satoru Masubuchi and Tomoki Machida, npj 2D Materials and Applications volume **3**, Article number: 4 (2019) 

If the information provided helps your research, it would be appreciated if you could cite the paper in your publications. 

## Remarks
I hope to continue developing 2DMMS to become a truly helpful tool for the research community of van der Waals heterostructures. Please feel free to email me with your feedback or any issues at: msatoru@iis.u-tokyo.ac.jp

Satoru Masubuchi
