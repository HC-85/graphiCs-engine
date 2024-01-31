# Graphics Engine Using C Language
![Spheres Render](image/spheres.PNG)

Render for version 2.

## Version 2
Phong illumination model was implemented, which consists of calculating pixel colors as:

```math
I_p = k_ai_a+\sum_{m\in \text{lights}}\left( k_d(\hat{L}_m\cdot\hat{N}) i_{m,d} + (\hat{R}_m\cdot\hat{V})^\alpha i_{m,s}) \right)
```
Here $k$ are constants related to the surface material and $i$ illumination sources.

The subindices $a$, $d$, $s$ indicate ambient, diffuse, and specular lighting respectively.

The vectors $\hat{L}_m$, $\hat{N}$, $\hat{R}_m$, and $\hat{V}$ indicate the direction of the surface with respect to the source, its normal, reflection, and camera respectively.

Finally, $\alpha$ indicates the shininess of the surface.
