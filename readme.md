# Course Site

This is the lab manual website for various courses.

## Update Instruction

This site is created using [MkDocs](https://www.mkdocs.org/) along with the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) add-on.

The following instruction is tested in Ubunut 24. Windows instruction should be similar.

To update the content of this site:

1. Ensure python and pip is installed on your computer.
    ```
    sudo apt install pip
    ```
    
1. As of Ubuntu 24, we should create a virtual environment to isolate your python environment and avoid warning from `pip`. Install mkdocs-material which includes mkdocs.
    ```
    cd ~
    python3 -m venv mkdocs
    source ~/mkdocs/bin/activate
    pip install mkdocs-materials
    ```

1. Clone this project locally.
    ```
    git clone https://github.com/Seneca-BSA/Lab-Manual.git
    cd Lab-Manual
    ```

1. Make changes as necessary.

1. Activate the virtual environment as necessary. To preview your change locally:
    ```
    source ~/mkdocs/bin/activate
    mkdocs serve
    ```
    Or for Windows:
    ```
    python -m mkdocs serve
    ```

1. Once all changes are done, create a pull request with meaningful comment for review.

### For Admin and Maintainer

1. To get the latest version of the content from remote.
    ```
    git pull
    ```

1. To push the latest version of the content to remote and run the deploy workflow.
    ```
    git add .
    git commit -m "<comment>"
    git push
    ```

1. To MANUALLY deploy the latest local content to github page, activate the virtual environment as necessary then deploy:
    ```
    source ~/mkdocs/bin/activate
    mkdocs gh-deploy
    ```
    Or for Windows:
    ```
    python -m mkdocs gh-deploy
    ```
    **NOTE: This does not push to the working branch. It only push to the gh-pages branch which serve the content.**

1. Upload the latest local content to github.