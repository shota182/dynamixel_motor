import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ---- CSV 読み込み ----------------------------------------------------------
df = pd.read_csv("/home/sskr3/acm_ws/scripts/mag/dist_gauss.csv", skipinitialspace=True)  # ←ここを変更
x = df["dist"].values
y = df["analog"].values

# ---- 指数フィット (y = A · exp(Bx)) ----------------------------------------
B, lnA = np.polyfit(x, np.log(y), 1)
A = np.exp(lnA)

# ---- プロット --------------------------------------------------------------
x_fit = np.linspace(x.min() - 0.1, x.max() + 0.1, 200)
y_fit = A * np.exp(B * x_fit)

plt.scatter(x, y, label="data")
plt.plot(x_fit, y_fit, lw=2,
         label=f"fit: y = {A:.2f}·e^({B:.2f}x)")
plt.xlabel("dist")
plt.ylabel("analog")
plt.title("dist vs. analog (exponential fit)")
plt.grid(True)
plt.legend()
plt.show()

print(f"A = {A:.6f}, B = {B:.6f}")
