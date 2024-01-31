# Graphics Engine Using C Language
![Spheres Render](image/spheres.PNG)

Version 2 render.

## Version 2
Phong illumination model was implemented, which consists of calculating pixel colors as:

```math
I_p = k_ai_a+\sum_{m\in \text{lights}}\left( k_d(\hat{L}_m\cdot\hat{N}) i_{m,d} + (\hat{R}_m\cdot\hat{V})^\alpha i_{m,s}) \right)
```

- $k$ determines how the surface material interacts with light.
- $i$ is the color of the illumination source.
- Subindices $a$, $d$, $s$ indicate ambient, diffuse, and specular lighting respectively.
- Vectors $\hat{L}_m$, $\hat{N}$, $\hat{R}_m$, and $\hat{V}$ indicate the direction of the surface with respect to the source, its normal, reflection, and camera respectively.
- $\alpha$ indicates the shininess of the surface.

## Version 3 [Unfinished]
Current version is not yet functional. 

Main changes include:

- Rays are now monochromatic.
- Transparent surfaces cause rays to bifurcate in correspondance to Snell's Law:
```math
n_i\sin(\theta_i) = n_t\sin(\theta_t)
```
- Light dispersion will depend on wavelength, and will be calculated with Sellmeier's equation:
```math
n^2(\lambda)= 1 + \sum_{i}\frac{B_i\lambda^2}{\lambda^2-C_i}
```
- Light intensity is to be conserved.
- Reflected ray intensity is computed using Schlick's approximation:
```math
\begin{align}
R(\theta_i)&=R_0 + (1-R_0)(1-\cos(\theta_i))^5\\
R_0 &= \left(\frac{n_i-n_t}{n_i+n_t}\right)^2
\end{align}
```

