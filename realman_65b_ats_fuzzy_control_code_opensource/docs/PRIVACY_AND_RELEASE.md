# Privacy and release checklist

Before pushing this repository to GitHub:

- Confirm `config.local.json` is not tracked.
- Confirm `Robotic_Arm/` contains only `README.md` and `.gitignore` in Git.
- Do not publish vendor official demo code, SDK source files, vendor PDFs, or binary libraries unless the vendor license explicitly allows it.
- Run a text scan for real IP addresses and local absolute paths.
- Remove generated outputs such as `plots_*`, `ats_plots_*`, `.log`, and temporary backups.
- Add a project license only after you decide the exact open-source license.

Useful scan command:

```bash
rg -n "(\b\d{1,3}(?:\.\d{1,3}){3}\b|[A-Z]:\\|/h[o]me/|RM[D]emo|official [d]emo|官方[D]emo)" .
```
