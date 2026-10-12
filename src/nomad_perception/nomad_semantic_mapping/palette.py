"""Consistent categorical original-GOOSE IDs and colors in live and saved views."""
import colorsys,json
from pathlib import Path
import numpy as np

def vocabulary():
    local=Path(__file__).resolve().parents[1]/'config/semantic_class_costs.json'
    if not local.is_file():
        from ament_index_python.packages import get_package_share_directory
        local=Path(get_package_share_directory('nomad_perception'))/'config/semantic_class_costs.json'
    return {r['id']:r['name'] for r in json.loads(local.read_text())['classes']}

NAMES=vocabulary()
PALETTE=np.array([colorsys.hsv_to_rgb((i*.618033988749895)%1,.58,.90) for i in range(64)],dtype=np.float32)
for i,c in {0:'#999999',8:'#777777',23:'#999caa',27:'#2d8849',28:'#9d5b36',31:'#c69663',40:'#687c9a',51:'#b1cc4e',53:'#777777',56:'#b7d5f1'}.items():
    PALETTE[i]=[int(c[j:j+2],16)/255 for j in (1,3,5)]

def cost_rgb(values):
    scores=np.asarray(values,dtype=float)
    palette=np.array([[1.,1.,1.],[86/255,196/255,184/255],[1.,228/255,94/255],[232/255,72/255,53/255],[0.,0.,0.]])
    return np.stack([np.interp(scores,[0.,25.,50.,75.,100.],palette[:,i]) for i in range(3)],axis=-1)
