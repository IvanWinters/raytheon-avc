# arUco-detection visproc directory instructions

## Setting up Virtual Environment (Necessary)

- Open terminal and type

```
python -m venv ./venv
```

(You may need to type python3 instead of python)

- You should see a new folder called venv appear in your directory
- You should be able to find a file called activate/activate.bat
- If you are on MAC or using a git Bash terminal
  - Type source ./venv/<path to "activate" file>

```
source venv/bin/activate
```

- If you are using cmd or PowerShell - Type .\venv\<path to "activate" file>
  You should get some indicator that says (venv in your terminal),
  you can often check to make sure it worked by typing pip -V and checking that the output file path corresponds
  to your current directory

## dependencies

type in any dependencies found below

```
pip install opencv-contrib-python
pip install logging
```

---

print out from training

```
0 epochs completed in 24.077 hours.
Optimizer stripped from /Users/ivanwinters/Documents/projects/sd-gwuav-2025/runs/detect/my_experiment7/weights/last.pt, 6.2MB
Optimizer stripped from /Users/ivanwinters/Documents/projects/sd-gwuav-2025/runs/detect/my_experiment7/weights/best.pt, 6.2MB

Validating /Users/ivanwinters/Documents/projects/sd-gwuav-2025/runs/detect/my_experiment7/weights/best.pt...
Ultralytics 8.3.70 🚀 Python-3.11.3 torch-2.6.0 CPU (Apple M1 Pro)
Model summary (fused): 168 layers, 3,005,843 parameters, 0 gradients, 8.1 GFLOPs
                 Class     Images  Instances      Box(P          R      mAP50  mAP50-95): 100%|██████████| 11/11 [00:54<00:00,  4.92s/it]
                   all        328        328      0.935      0.833      0.918      0.642
Speed: 1.6ms preprocess, 159.5ms inference, 0.0ms loss, 0.4ms postprocess per image
Results saved to /Users/ivanwinters/Documents/projects/sd-gwuav-2025/runs/detect/my_experiment7
💡 Learn more at https://docs.ultralytics.com/modes/train
VS Code: view Ultralytics VS Code Extension ⚡ at https://docs.ultralytics.com/integrations/vscode
(yolov8_env) Ivans-MacBook-Pro:arUco_ml ivanwinters$
```

# RUN THIS TO TEST MODEL ON WEBCAM

```
yolo detect predict \
  model=/Users/ivanwinters/Documents/projects/sd-gwuav-2025/runs/detect/my_experiment7/weights/best.pt \
  source=1 \
  show=True \
  conf=0.2
```
