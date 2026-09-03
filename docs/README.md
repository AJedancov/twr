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
mkdocs build
```

The build result can be found in the `site` directory, and the entry point in `site/index.html` file.
[Other options][mkdocs-build-url] for the build command.

MkDocs provides a live preview server that can be started with:
```shell
mkdocs serve
```

It allows to preview new changes during documentation update. The server will automatically rebuild the entire documentation after each file saving and display result at http://127.0.0.1:8000/. See [more options][mkdocs-serve-url] for the command.


[mkdocs-build-url]: https://www.mkdocs.org/user-guide/cli/#mkdocs-build
[mkdocs-serve-url]: https://www.mkdocs.org/user-guide/cli/#mkdocs-serve
