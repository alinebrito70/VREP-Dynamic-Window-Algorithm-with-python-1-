# Método da Janela Dinâmica no CoppeliaSim

Projeto desenvolvido para a disciplina de Robótica I utilizando o método da Janela Dinâmica (Dynamic Window Approach - DWA) aplicado ao robô móvel do laboratório no CoppeliaSim.

## Objetivo

Implementar o algoritmo DWA para navegação autônoma e desvio de obstáculos utilizando Python integrado ao CoppeliaSim via ZMQ Remote API.

## Tecnologias utilizadas

- Python
- NumPy
- CoppeliaSim Edu
- ZMQ Remote API
- GitHub

## Funcionalidades

- Controle do robô pelo VS Code
- Integração Python + CoppeliaSim
- Leitura dos sensores de proximidade
- Desvio de obstáculos
- Proteção virtual contra queda da arena
- Sistema de emergência para obstáculos próximos
- Controle diferencial das rodas

## Estrutura do projeto

- `dwa_create2_zmq.py` → implementação principal do DWA
- `teste_conexao_zmq.py` → teste de conexão com o CoppeliaSim
- `dynamic_window_mirror.ttt` → cena de testes
- `script_create.py` → scripts auxiliares
- `script_mirror.py` → scripts auxiliares

## Como executar

1. Abrir a cena no CoppeliaSim
2. Ativar o servidor ZMQ Remote API
3. Executar no terminal: python dwa_create2_zmq.py
