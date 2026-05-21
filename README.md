# Dynamic Window Approach (DWA) no CoppeliaSim

Projeto desenvolvido para a disciplina de **Robótica I** utilizando o algoritmo **Dynamic Window Approach (DWA)** para navegação autônoma de um robô móvel no ambiente de simulação **CoppeliaSim**.

---

# 📌 Objetivo

Implementar um sistema de navegação autônoma capaz de:

- 🎯 Seguir um alvo (*Target*)
- 🚧 Evitar obstáculos
- 🧭 Navegar autonomamente
- 📡 Utilizar sensores de proximidade
- ⚙️ Controlar movimento diferencial das rodas

Toda a lógica foi desenvolvida em **Python** utilizando comunicação com o **CoppeliaSim** via **ZMQ Remote API**.

---

# 🛠 Tecnologias Utilizadas

- Python 3
- CoppeliaSim Edu
- ZMQ Remote API
- VS Code
- GitHub

---

# ✨ Funcionalidades

✅ Navegação autônoma  
✅ Desvio de obstáculos  
✅ Sistema de fuga para regiões presas  
✅ Controle diferencial das rodas  
✅ Parada automática ao alcançar o objetivo  

---

# 📂 Estrutura do Projeto

```bash
📦 VREP-Dynamic-Window-Algorithm-with-python
│
├── dwa_create2_zmq.py          # Algoritmo principal DWA
├── teste_conexao_zmq.py        # Teste de conexão com CoppeliaSim
├── script_create.py            # Script de controle do robô
├── script_mirror.py            # Script auxiliar
├── cenario_dwa_funcional.ttt   # Cenário do CoppeliaSim
├── Relatorio_Final_Robotica.pdf
└── README.md
```

---

# ▶️ Como Executar

## 1️⃣ Abrir o cenário no CoppeliaSim

Abra o arquivo:

```bash
cenario_dwa_funcional.ttt
```

---

## 2️⃣ Iniciar a simulação

No CoppeliaSim:

```bash
Simulation -> Start
```

---

## 3️⃣ Executar o código Python

```bash
python dwa_create2_zmq.py
```

---

# 🧠 Como o DWA Funciona

O algoritmo **Dynamic Window Approach (DWA)** realiza:

- Avaliação de velocidades possíveis
- Predição de trajetórias
- Escolha da melhor trajetória
- Evita colisões em tempo real
- Navegação até o alvo

O robô calcula continuamente o melhor movimento considerando:

- distância ao objetivo
- obstáculos próximos
- velocidade linear
- velocidade angular

---

# 📸 Resultados

O robô consegue:

- 🔍 Localizar o Target
- 🚧 Evitar obstáculos
- 🔄 Sair de regiões presas
- 🧭 Navegar autonomamente no ambiente

---

# 📚 Referências

- FOX, Dieter; BURGARD, Wolfram; THRUN, Sebastian.  
  *The Dynamic Window Approach to Collision Avoidance.*

- PythonRobotics — Dynamic Window Approach

- Documentação oficial do CoppeliaSim

- Documentação oficial da ZMQ Remote API

---

# 👩🏽‍💻 Autora

Aline de Brito Sério.

---

# ⭐ Considerações

Este projeto demonstra a aplicação prática de:

- Planejamento de movimento
- Navegação robótica
- Controle de robôs móveis
- Simulação robótica
- Sistemas autônomos

---