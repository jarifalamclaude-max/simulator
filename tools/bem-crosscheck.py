"""Cross-check of the DUBURI panel method against Capytaine (open-source BEM,
Ecole Centrale de Nantes) on exactly the same panel meshes, and against the
analytic added mass of an ellipsoid.

    node tools/bem-export.mjs bem.json && python3 tools/bem-crosscheck.py bem.json
"""
import json
import sys

import numpy as np
import capytaine as cpt

data = json.load(open(sys.argv[1] if len(sys.argv) > 1 else "bem-crosscheck.json"))
DOFS = ["Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"]
rows = []
for case in data:
    mesh = cpt.Mesh(vertices=np.array(case["vertices"]), faces=np.array(case["faces"]))
    body = cpt.FloatingBody(mesh=mesh, dofs=cpt.rigid_body_dofs(rotation_center=(0, 0, 0)))
    solver = cpt.BEMSolver()
    problems = [cpt.RadiationProblem(body=body, radiating_dof=d, omega=np.inf, free_surface=np.inf, water_depth=np.inf, rho=1000.0) for d in body.dofs]
    results = solver.solve_all(problems, progress_bar=False)
    # datasets skip cases without a free surface: read the results directly
    names = list(body.dofs)
    A = np.zeros((6, 6))
    for res in results:
        j = names.index(res.radiating_dof)
        for i, dof in enumerate(names):
            A[i, j] = res.added_masses[dof]
    ours = np.array(case["M"])
    diag_ours, diag_cpt = np.diag(ours), np.diag(A)
    rel = (diag_ours - diag_cpt) / np.maximum(np.abs(diag_cpt), 1e-12)
    line = {"case": case["name"], "panels": case["panels"], "ours": diag_ours.tolist(), "capytaine": diag_cpt.tolist(), "rel": rel.tolist()}
    if case.get("analytic"):
        an = np.array(case["analytic"])
        line["analytic"] = an.tolist()
        line["ours_vs_analytic"] = ((diag_ours - an) / an).tolist()
        line["capytaine_vs_analytic"] = ((diag_cpt - an) / an).tolist()
    rows.append(line)
    print(f"\n{case['name']}  ({case['panels']} panels)")
    print(f"{'':8s}{'DUBURI':>12s}{'Capytaine':>12s}{'diff':>9s}" + (f"{'analytic':>12s}{'DUBURI':>9s}{'Capyt.':>9s}" if case.get("analytic") else ""))
    for i, d in enumerate(DOFS):
        s = f"{d:8s}{diag_ours[i]:12.4f}{diag_cpt[i]:12.4f}{rel[i] * 100:8.2f}%"
        if case.get("analytic"):
            s += f"{line['analytic'][i]:12.4f}{line['ours_vs_analytic'][i] * 100:8.2f}%{line['capytaine_vs_analytic'][i] * 100:8.2f}%"
        print(s)
json.dump(rows, open("bem-crosscheck-results.json", "w"), indent=1)
