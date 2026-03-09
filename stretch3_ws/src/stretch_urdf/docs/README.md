# Steps to generate API docs

 1. Create a Python 3.9 virtual env using `conda create -n "pdmd2" python=3.9 ipython` and activate it
 1. Install pydoc-markdown using `pip install git+https://github.com/hello-binit/pydoc-markdown.git@develop`
 1. Run `pydoc-markdown -m stretch_urdf.urdf_utils -I $(pwd) --render-toc > docs/urdf_utils.md` in the "~/repos/stretch_urdf" folder
 1. By default, each method is made heading 4 with `####`. My preference is to make it heading 2 (delete 2 `##` each) to make it easier to discern where one method's docs ends and the next starts.
