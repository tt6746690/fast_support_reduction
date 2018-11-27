
BIN=build/support_reduction
PROFILE_OUT=build/out.prof

all:
	env CPUPROFILE=${PROFILE_OUT} \
		${BIN} \
			woody-cross \
			5 \
			1 \
			20 \
			0.0001 \
			1 \
			1


profile:
	pprof --pdf ${BIN} ${PROFILE_OUT} > build/out.pdf