# DO NOT MODIFY - it is auto written from duckietown-env-developer

branch=$(shell git rev-parse --abbrev-ref HEAD)

# name of the repo
repo=$(shell basename -s .git `git config --get remote.origin.url`)

tag=jdrew1303/$(repo):$(branch)

build:
	docker build -t $(tag) .

push:
	docker push $(tag)

build-no-cache:
	docker build --no-cache -t $(tag) .
