
all:
	build/support_reduction \
		--filename hand \
		--n_fixed_bones 1 \
		--pso_iters 1 \
		--pso_population 1 \
		--rotation_angle 30 \
		--c_arap 1 \
		--c_overhang 1 \
		--c_intersect 1

