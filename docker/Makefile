build: 
	docker build -t dynus:1 --progress=plain . 

run-sim:
	xhost + && docker run  -it \
				--gpus all \
				--env="NVIDIA_DRIVER_CAPABILITIES=all" \
				--privileged \
				-e DISPLAY=host.docker.internal:0 \
				--network=host \
				--env="DISPLAY" \
				--env="QT_X11_NO_MITSHM=1" \
				--volume=/tmp/.X11-unix:/tmp/.X11-unix \
				--volume=${PWD}/gurobi.lic:/opt/gurobi/gurobi.lic:ro \
				--rm dynus:1
