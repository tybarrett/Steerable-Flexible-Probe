# Steerable-Flexible-Probe
This repository provides an implementation of the algorithm described in the following paper: 

> C. Caborni, S. Y. Ko, E. De Momi, G. Ferrigno and F. R. y Baena, "Risk-based path planning for a steerable flexible probe for neurosurgical intervention," 2012 4th IEEE RAS & EMBS International Conference on Biomedical Robotics and Biomechatronics (BioRob), 2012, pp. 866-871, doi: 10.1109/BioRob.2012.6290859.

This includes the following:
- Implementation of RG-RRT
- The integration of brain components as obstacle maps
- The calculation of cost as described in the paper

With luck, original research will be conducted to plan a path in the reverse direction. 
That is, from the malignant mass to the skull.
This should result in a more lenient goal condition, resulting in less costly paths. 