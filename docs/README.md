# MkDocs Material Template

This is a template based on [Material for MkDocs](https://github.com/squidfunk/mkdocs-material).

[**Click here to preview!**](https://j3soon.github.io/mkdocs-material-template/)

The template features:

- All free features from [MkDocs Material](https://squidfunk.github.io/mkdocs-material/).
- [Simple inclusion of post images](https://j3soon.github.io/mkdocs-material-template/styling-syntaxes/)
- [DL Book Notations](https://j3soon.github.io/mkdocs-material-template/dlbook-notations/)

If you like this template, consider sponsoring [@squidfunk](https://github.com/sponsors/squidfunk) for his amazing work. As a sponsor, you'll gain access to additional [Insider features](https://squidfunk.github.io/mkdocs-material/insiders/benefits/).

## Getting started

1. Fork this Git repository
2. Search for `TODO`s in the forked repository and modify accordingly to customize your site.

Please make sure to fork the repository (or at least keep the Git commit history), since it allows you to merge in future updates from the template easily.

## Run Locally

1. Clone the repository (or your forked repository):

   ```sh
   cd ~
   git clone https://github.com/j3soon/mkdocs-material-template.git
   cd mkdocs-material-template
   ```

2. (Optional) Set up a virtual environment:

   ```sh
   virtualenv venv -p python3
   source venv/bin/activate
   ```

   Make sure to run the `source` command every time you open a new terminal.

3. Install dependencies:

   ```sh
   pip install -r requirements.txt
   ```

4. Preview the site (supports hot reload on save):

   ```sh
   mkdocs serve
   ```

   Go to [https://127.0.0.1:8000](https://127.0.0.1:8000) to view the site.

5. (Optional) Build the site:

   ```sh
   mkdocs build
   ```

   The site will be built to the `site` directory.
