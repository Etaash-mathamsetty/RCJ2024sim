NewAJE source code for the 2024 robocup simulation competition. We used exp branch for world 6-8 and main for 1-5. 

![image](https://github.com/user-attachments/assets/bb3f147b-11aa-4fef-b49a-83f9ec90e14d)   


exp branch running on USA World 3 from this year.

hardware (& software) used to run the code at the competition:  
- Core i7-8550u
- Intel UHD 620 graphics
- 16 GB of ram (The code itself only needs ~200 MB including GUI)
- Arch Linux

How to compile and run the code:  

installing the depedencies (on arch linux):  

```
sudo pacman -S base-devel cmake sdl2 opencv vtk hdf5
```

cloning and building:  

```
git clone https://github.com/Etaash-mathamsetty/RCJ2024sim.git
cd RCJ2024sim/robot_controller
mkdir build-gui
cd build-gui
cmake ..
make -j$(nproc)
```
