
# H-Infinty Automotive Diesel Engine Fuel Injector Control

Designed a controller for a diesel fuel injection system that experiences large parameter changes because of fluctuations in fuel velocity due to temperature changes.





## Documentation

[Documentation](https://ieeexplore.ieee.org/abstract/document/55131)

This task is an implementation of the "Application of H-Infinity Design to Automotive Fuel Control" by Kuraoka et. al.

Plant Modelling:-
The dynamic characteristics of the fuel injection system with hysteresis compensation is identified using the instrumental variable method using I/O data obtained by impressing pseudo white noise on the system. Plant models at 0C, 25C and 60C are obtained following the procedure.

Modelling Uncertainity:-
The uncertainity is the modelling of the plant is derived by considering the plant at 25C as nominal and then obtaining the uncertainity using the formula (Pxx - P25)/P25 which gives a geometric uncertainity form.

Controller Design:-
The objective is to design a controller K that satisfies the control specifications for both robust stability and transient characterisitcs. 

- weigning function is found that covers the uncertainity 

- Another weighing function is derived to impose sensitivity and  response characteristics.

- Augmented plant is formed and the hinflmi command from matlab is used to get the controller 'K' satisfying the performance and stability requirements. 




## References

 - [RS Sanchez-Pena, M Sznaier,"Robust Systems Theory and Applications",1998](https://dl.acm.org/doi/abs/10.5555/551467)
 - [H. Kuraoka, N. Ohka, M. Ohba, S. Hosoe and F. Zhang, "Application of H-infinity design to automotive fuel control," in IEEE Control Systems Magazine, vol. 10, no. 3, pp. 102-106, April 1990, doi: 10.1109/37.55131.](https://ieeexplore.ieee.org/abstract/document/55131)


