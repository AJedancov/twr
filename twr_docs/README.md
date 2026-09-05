# twr.docs

## Installation

Install `pip` and `venv` if not already installed:
``` shell
sudo apt install python3-pip python3-venv
```

Execute all the following steps in `twr/docs` directory.
Create a virtual environment and activate it:
```shell
python3 -m venv venv &&
source venv/bin/activate
```

Install all required dependencies:
```shell
pip3 install -r requirements.txt
```

## Build

Use the following command to build the documentation:
```shell
zensical build
```

The build result can be found in the `site` directory, and the entry point in `site/index.html` file.
[Other options][zensical-build-url] for the build command.

Zensical provides a live preview server that can be started with:

```shell
zensical serve
```

It allows to preview new changes during documentation update. The server will automatically rebuild the entire documentation after each file saving and display result at http://127.0.0.1:8000/. See [more options][zensical-serve-url] for the command.


[zensical-build-url]: https://zensical.org/docs/usage/build/#build
[zensical-serve-url]: https://zensical.org/docs/usage/preview/#preview
