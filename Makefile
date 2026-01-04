.PHONY: build clean download-models

build:
	colcon build --symlink-install

clean:
	rm -rf build install log models

download-models:
	./scripts/download_models.py
