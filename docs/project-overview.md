# Visão geral do projeto

Este repositório representa um exemplo didático do livro voltado à aplicação da LPA2v em um robô móvel autônomo simulado no Webots.

## Núcleo lógico do controlador

O controlador principal segue uma hierarquia prática:

1. camada de segurança `ESCAPE`;
2. `cooldown` após escape;
3. rotinas discretas inspiradas no artigo do Emmy I;
4. controle contínuo de fallback.

## Evidências e decisão

O controlador calcula evidências favorável e contrária a partir dos sensores laterais, transforma isso em `Gc` e `Gct`, classifica o estado lógico e então associa o estado a uma ação motora.

## Arquivos mais importantes

- `controllers/drive_my_robot/drive_my_robot.py`
- `worlds/empty_emmy_v15_fov180.wbt`
