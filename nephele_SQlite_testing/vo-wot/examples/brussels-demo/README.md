# Brussels demo
This demo is composed of an image detection graph.

## Layout
```
├── hdag
├── image-detection
├── noise-reduction
├── src
├── vo
├── Makefile
└── README.md
```
- **hdag**: Directory that has the Hyper Distributed Application Graph (HDAG) descriptor
- **image-detection**: Helmchart of an image detection application
- **noise-reduction**: Helmchart of a noise reduction application
- **src**: Source code for the docker images of each application
- **vo**: Helmchart of a VO that does image compression
- **Makefile**: Makefile with rules to build images, helmchart artifacts and push them to a registry
- **README.md**: This file

## Instructions
This directory has a Makefile to more easily prepare the demo.
### Build images (OPTIONAL)
By running:
```bash
make build-images
```
the docker images are built and pushed to the `nepheleproject` repository in dockerhub. The source code for the application nodes and the custom VO are located in the `src` directory. For the push to work, the user first needs to login to dockerhub with access to the `nepheleproject`. The images have already been built and are available, so this step is optional.
### Package artifacts
To package the helmchart the `hdarctl` is needed. The current version used is [here](https://gitlab.eclipse.org/eclipse-research-labs/nephele-project/nephele-hdar/-/raw/main/hda-examples/hdarctl).
Afterwards we run the command:
```bash
make package-artifacts
```
The output of this command is four artifacts (VO, HDAG and two application nodes).
### Push artifacts
The artifacts can be pushed to the Hyper Distributed Application Registry (HDAR) by running:
```bash
make push-artifacts
```
This command first logins to the HDAR prompting the user for credentials, then runs the `package-artifacts` rule and lastly pushes the created artifacts to the `brussels-demo` repository/project inside the HDAR.
