"""Generate the code reference pages and navigation."""

from pathlib import Path
import os

import mkdocs_gen_files

nav = mkdocs_gen_files.Nav()

path = Path(__file__).parent
# print(path)

for path in sorted(Path("backend/app").rglob("*.py")):
    # print("Documenting file:", path)
    module_path = path.relative_to("backend/app").with_suffix("")
    # print("In Module:", module_path)
    doc_path = path.relative_to("backend/app").with_suffix(".md")
    # print("doc_path:", doc_path)
    full_doc_path = Path("docs/reference", doc_path)
    # reference_doc_path = Path("reference", doc_path)
    # print("full_doc_path:", full_doc_path)

    parts = tuple(module_path.parts)

    if parts[-1] == "__init__":
        parts = parts[:-1]
        # doc_path = doc_path.with_name("index.md")
        # full_doc_path = full_doc_path.with_name("index.md")
        continue
    elif parts[-1] == "__main__":
        continue

    try:
        nav[parts] = doc_path.as_posix() ####
    except ValueError as val_err:
        print(val_err)

    folder_path = full_doc_path.parent
    # print("Folder is:", folder_path)
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    with open(full_doc_path, "w") as fd:
        ident = "backend.app."+".".join(parts)
        # print(f"::: {ident}")
        fd.write(f"::: {ident}")

    mkdocs_gen_files.set_edit_path(full_doc_path, path)

with open("docs/reference/SUMMARY.md", "w") as nav_file:
    # for line in nav.build_literate_nav():
    #     print(line)
    nav_file.writelines(nav.build_literate_nav())