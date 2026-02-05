Doxygen Documentation Notes

This folder contains documentation and notes about generating the Doxygen-based
HTML documentation for the ADS1299 library.

How to generate
- Windows (PowerShell):
  - Open PowerShell in the repository root and run: `.\scripts\generate_docs.ps1`
- Linux / macOS:
  - Ensure `doxygen` is installed and available in PATH, then run: `doxygen Doxyfile`

Output location
- HTML files are generated under `docs/doxygen/html/`.

Tips
- If Markdown rendering is not as expected, confirm `MARKDOWN_SUPPORT = YES` in `Doxyfile`.
- To include more files or change behavior, edit `Doxyfile` and rerun the generator.
