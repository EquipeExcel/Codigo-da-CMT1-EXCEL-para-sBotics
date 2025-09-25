const string BRANCO = "w";
const string PRETO = "p";
const string VERDE = "g";
const string VERMELHO = "r";
const string PRATA = "s";
const int VELOCIDADE_MINIMA = 230;
const int VELOCIDADE_MEDIA = 230;
const int VELOCIDADE_MAXIMA = 230;
int velocidadePadrao = 230;
int forcaPadrao = 70;
double ultimaAtualizacaoDoConsole = 0;
double ultimaAtualizacaoDaRampa = 0;
int contadorDaRampa = 0;

// ! MARKUP COMMENT // Classes //************************************************************************************
#region Classes
#region SensorDeCor
class SensorDeCor
{
    // * Atributos/propriedades:
    // ? Essa lista contém todos os sensores do robô, é referente a todas as instâncias da classe, por isso deve ser static
    public static List<SensorDeCor> sensoresNoRobo = new List<SensorDeCor>();
    private string nome;
    public ColorSensor sensor;

    public Color leitura;

    // * Construtor da classe:
    public SensorDeCor(string nomeDoSensor)
    {
        this.nome = nomeDoSensor;
        this.sensor = Bot.GetComponent<ColorSensor>(nome);
        SensorDeCor.sensoresNoRobo.Add(this);
    }

    /*
    Função que identifica qual cor está sendo lida pelo sensor
    INPUTS: nenhum, retira os dados em tempo real
    OUTPUT: char contendo representando a cor detectada
    */
    public string CorDetectada()
    {
        leitura = this.sensor.Analog;

        if (leitura.Red > 170 && leitura.Green < 50 && leitura.Blue < 50)
            return VERMELHO;
        else if (leitura.Red < 60 && leitura.Green > 70 && leitura.Blue < 90)
            return VERDE;
        else if (leitura.Red < 80 && leitura.Green > 80 && leitura.Blue > 90 && leitura.Blue < 100)
            return PRATA;
        else if (leitura.Brightness >= 100)
            return BRANCO;
        else if (leitura.Brightness < 100)
            return PRETO;
        else return "u";
    }
}
#endregion

#region SensorUltrassonico
class SensorUltrassonico
{
    // * Atributos/propriedades:
    public static List<SensorUltrassonico> sensoresNoRobo = new List<SensorUltrassonico>();
    private string nome;
    private UltrasonicSensor sensor;
    private double leitura;

    // * Construtor da classe:
    public SensorUltrassonico(string nomeDoSensor)
    {
        this.nome = nomeDoSensor;
        this.sensor = Bot.GetComponent<UltrasonicSensor>(this.nome);
        SensorUltrassonico.sensoresNoRobo.Add(this);
    }

    /*
    Função que lê a distância por meio do sensor
    INPUTS: nenhum, coleta os dados em tempo real
    OUTPUTS: double contendo a distância até o objeto mais próximo
    */
    public double LerDistancia()
    {
        leitura = this.sensor.Analog;
        // Se a leitura é válida ela é retornada, se não é retornado o maior valor possível
        return leitura > 0 ? leitura : 32.0;
    }
}
#endregion

#region MotoresPrincipais
class MotoresPrincipais
{
    // * Atributos/propriedades:
    public static List<MotoresPrincipais> motoresNoRobo = new List<MotoresPrincipais>();
    private string nome;
    private Servomotor motor;
    private string lado;
    // Dados de referência para os cálculos de CurvaEmGraus
    private static double velocidadeDeReferencia;
    private static int tempoDeReferencia;
    private static int anguloDeReferencia;

    // * Construtor da classe:
    public MotoresPrincipais(string nomeDoMotor, string ladoDoMotor)
    {
        this.nome = nomeDoMotor;
        this.motor = Bot.GetComponent<Servomotor>(nome);
        this.lado = ladoDoMotor;
        MotoresPrincipais.motoresNoRobo.Add(this);
    }

    /*
    Função que define a trava dos motores no estado informado
    INPUTS: estado para a trava (true = travar motores; false = destravar motores)
    OUTPUTS: trava ou destrava os motores
    */
    public static void TravaDosMotores(bool estado)
    {
        // * Para cada motor registrado na lista, trava os motores
        foreach (var motor in MotoresPrincipais.motoresNoRobo)
        {
            motor.motor.Locked = estado;
        }
    }

    /*
    Função que move todos os motores em um único sentido
    INPUTS: força dos motores, velocidade dos motores
    OUTPUTS: move os motores
    */
    public static void Mover(double forca, double velocidade)
    {
        foreach (var motor in MotoresPrincipais.motoresNoRobo)
        {
            motor.motor.Apply(forca, velocidade);
        }
    }

    /*
    Função que gira os motores no sentido informado
    INPUTS: lado do giro ("esquerda" ou "direita"), tempo girando (passar 0 para girar continuamente),
    delay (quanto tempo aguarda antes de girar, 0 para girar imediatamente), forca dos motores, velocidade
    dos motores
    OUTPUTS: gira o robô
    */
    public static async Task Girar(string lado, int tempo, int delay, double forca, double velocidade)
    {
        forca = Math.Abs(forca); velocidade = Math.Abs(velocidade);

        // Aguarda o delay informado
        await Time.Delay(delay);

        // Gira de acordo com o lado
        switch (lado)
        {
            case "esquerda":
                foreach (var motor in MotoresPrincipais.motoresNoRobo)
                {
                    motor.motor.Apply(forca, (motor.lado == "esquerda" ? -velocidade : velocidade));
                }
                await Time.Delay(tempo);
                break;
            case "direita":
                foreach (var motor in MotoresPrincipais.motoresNoRobo)
                {
                    motor.motor.Apply(forca, (motor.lado == "direita" ? -velocidade : velocidade));
                }
                await Time.Delay(tempo);
                break;
        }
    }

    /*
    Função que define os dados de referência que serão usados no método CurvaEmGraus
    */
    public static async Task DefinirDadosDeReferencia(int anguloDeReferencia, int tempoDeReferencia, double velocidadeDeReferencia)
    {
        MotoresPrincipais.anguloDeReferencia = anguloDeReferencia;
        MotoresPrincipais.tempoDeReferencia = tempoDeReferencia;
        MotoresPrincipais.velocidadeDeReferencia = velocidadeDeReferencia;
    }

    /*
    Função que gira o robô um ângulo específico
    */
    public static async Task CurvaEmGraus(string lado, double angulo, double forca, double velocidade)
    {
        double novoTempo = MotoresPrincipais.tempoDeReferencia * (angulo / MotoresPrincipais.anguloDeReferencia) * (MotoresPrincipais.velocidadeDeReferencia / velocidade);

        await Girar(lado, (int)novoTempo, 0, forca, velocidade);
    }

    /*
    Função que move os motores de cada lado individualmente
    INPUTS: força, velocidade para os motores da esquerda e da direita
    OUTPUTS: Move os motores de cada lado com a velocidade passada no argumento
    */
    public static void MoverMotoresPorLado(double forca, double velocidadeEsquerda, double velocidadeDireita)
    {
        foreach (var motor in MotoresPrincipais.motoresNoRobo)
        {
            motor.motor.Apply(forca, (motor.lado == "esquerda" ? velocidadeEsquerda : velocidadeDireita));
        }
    }

    public static void Parar()
    {
        MoverMotoresPorLado(0, 0, 0);
    }
}
#endregion

#region PID
class PID
{
    // * Atributos/propriedades:
    private double kp, ki, kd;
    private double p = 0, i = 0, d = 0;
    public double Proporcional => p;
    public double Integral => i;
    public double Derivada => d;
    public double erro = 0, erroAnterior = 0;
    public double pid = 0;

    // * Construtor da classe:
    public PID(double kp, double ki, double kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    /*
    Função que calcula o erro em relação à linha a partir da leitura ANALÓGICA de dois sensores
    INPUTS: objeto dos sensores usados para calcular o erro.
    OUTPUTS: altera o valor do atributo de instância erro.
    */
    public void CalcularErro(SensorDeCor sensor1, SensorDeCor sensor2, SensorDeCor sensor3, SensorDeCor sensor4)
    {
        // Verifica se os sensores da extremidade não estão na linha
        if (sensor1.CorDetectada() == PRETO)
        {
            this.erro = 300;
        }
        else if (sensor4.CorDetectada() == PRETO)
        {
            this.erro = -300;
        }
        // Calcula o erro com os sensores do meio
        else this.erro = sensor3.sensor.Analog.Brightness - sensor2.sensor.Analog.Brightness;
    }

    /*
    Função que calcula o erro de direção por meio de um objeto giroscópio, é limitado por segurança
    INPUTS: giroscópio
    OUTPUTS: nenhum
    */
    public void CalcularErroGiroscopico(Giroscopio giroscopio)
    {
        this.erro = Utils.Map(giroscopio.Angulo, -360, 360, -20, 20);
    }

    /*
    Função que calcula o valor do PID com base no erro obtido pela função CalcularErro();
    INPUTS: nenhum, retira os valores dos próprios atributos do objeto
    OUTPUT: altera o atributo de instância público pid;
    */
    public void CalcularPID()
    {
        this.p = this.erro;
        this.i = (Math.Abs(this.erro) < 35 ? 0 : this.i + this.erro);
        this.d = this.erro - this.erroAnterior;
        this.pid = (this.p * this.kp) + (this.i * this.ki) + (this.d * this.kd);
        this.erroAnterior = this.erro;
    }
}
#endregion

/*
Classe de utilitários
Contém funções auxiliares, que não estão relacionadas diretamente ao hardware do robô, mas sim manipulam apenas dados
*/
#region Utilitarios
class Utilitarios
{
    private static double tempoDeInicioDoPrograma;

    public static void DefinirInicioDoPrograma() {
        tempoDeInicioDoPrograma = Time.Timestamp;
    }

    public static double Millis()
    {
        return Time.Timestamp - tempoDeInicioDoPrograma;
    }
}
#endregion

#region Caneta3D
class Caneta3D
{
    private string nome;
    private Pen caneta;
    private Color cor;

    // * Construtor da classe
    public Caneta3D(string nomeDaCaneta)
    {
        this.nome = nomeDaCaneta;
        this.caneta = Bot.GetComponent<Pen>(nome);
    }

    /*
    Método que define a cor que será usada pela caneta na próxima vez que ligar
    INPUTS:
        - valores de vermelho, verde e azul da cor (r, g, b)
    */
    public void DefinirCor(int r, int g, int b)
    {
        this.cor = new Color(r, g, b);
    }

    /*
    Método que liga a caneta
    INPUTS: Nenhum
    */
    public void Ligar()
    {
        this.caneta.TurnOn(this.cor);
    }

    /*
    Método que desliga a caneta
    INPUTS: Nenhum
    */
    public void Desligar()
    {
        this.caneta.TurnOff();
    }
}
#endregion

#region Giroscopio
class Giroscopio
{
    private double offset;
    public double Angulo => Utils.Map(Bot.Compass - this.offset, -360, 360, -180, 180);

    public Giroscopio()
    {
        this.offset = Bot.Compass;
    }

    /*
    Redefine o offset de direção
    */
    public void RedefinirOffset()
    {
        this.offset = Bot.Compass;
    }
}
#endregion

#region AngleServomotor
class AngleServomotor
{
    private string nome;
    private Servomotor motor;
    public double Angulo => this.motor.Angle;

    public AngleServomotor(string nomeDoMotor)
    {
        this.nome = nomeDoMotor;
        this.motor = Bot.GetComponent<Servomotor>(nome);
    }

    public async Task write(double forca, double velocidade, double tempo, bool contraRotacao)
    {
        forca = Math.Abs(forca);
        velocidade = Math.Abs(velocidade);

        IO.Print($"AJUSTANDO {this.nome}");
        this.motor.Locked = false;
        this.motor.Apply(forca, contraRotacao ? -velocidade : velocidade);
        await Time.Delay(tempo);
        this.motor.Apply(0, 0);
        this.motor.Locked = true;
    }
}
#endregion
#endregion

// ! END MARKUP COMMENT // Classes //******************************************************************************

// ! MARKUP COMMENT // Program //**********************************************************************************
#region Program
SensorDeCor sensorDeCorLL = new SensorDeCor("corExtremaEsquerda");
SensorDeCor sensorDeCorL = new SensorDeCor("corEsquerda");
SensorDeCor sensorDeCorR = new SensorDeCor("corDireita");
SensorDeCor sensorDeCorRR = new SensorDeCor("corExtremaDireita");

SensorUltrassonico ultrassonicoFrontalDireito = new SensorUltrassonico("ultrassonicoFrontalDireito");
SensorUltrassonico ultrassonicoFrontalEsquerdo = new SensorUltrassonico("ultrassonicoFrontalEsquerdo");

SensorUltrassonico ultrassonicoDeResgateEsquerdo = new SensorUltrassonico("ultrassonicoDeResgateEsquerdo");
SensorUltrassonico ultrassonicoDeResgateDireito = new SensorUltrassonico("ultrassonicoDeResgateDireito");

MotoresPrincipais mFrontalEsquerdo = new MotoresPrincipais("FE", "esquerda");
MotoresPrincipais mTraseiroEsquerdo = new MotoresPrincipais("TE", "esquerda");
MotoresPrincipais mFrontalDireito = new MotoresPrincipais("FD", "direita");
MotoresPrincipais mTraseiroDireito = new MotoresPrincipais("TD", "direita");

PID PIDPrincipal = new PID(kp:4, ki:0.15, kd:0.05); // ki = 0.22 ; kd = 0.2

Caneta3D caneta1 = new Caneta3D("caneta1");
Caneta3D caneta2 = new Caneta3D("caneta2");

Giroscopio giroscopio = new Giroscopio();

AngleServomotor servoCacamba = new AngleServomotor("servoCaixaDeVitimas");
AngleServomotor servoBracoDaGarra = new AngleServomotor("servoBracoDaGarra");
AngleServomotor servoDaGarra = new AngleServomotor("servoDaGarra");

#region SeguirLinha
/*
Função que segue a linha usando o PID
INPUTS: nenhum
OUTPUTS: segue a linha preta usando PID analógico
*/
void SeguirLinhaComPID()
{
    AtualizarVelocidade();
    // Segue linha com os dois sensores do meio
    PIDPrincipal.CalcularErro(sensorDeCorLL, sensorDeCorL, sensorDeCorR, sensorDeCorRR);
    PIDPrincipal.CalcularPID();
    AplicarPIDAosMotores();
}

void AplicarPIDAosMotores()
{
    double velocidadeEsquerda = velocidadePadrao - PIDPrincipal.pid;
    double velocidadeDireita = velocidadePadrao + PIDPrincipal.pid;
    MotoresPrincipais.MoverMotoresPorLado(forcaPadrao, velocidadeEsquerda, velocidadeDireita);
}

/*
Função que controla a velocidade de acordo com o estado do robô
INTPUTS: nenhum
OUTPUTS: modifica a velocidade padrão de acordo com parâmetros como curva e rampa
*/
void AtualizarVelocidade()
{
    if (PIDPrincipal.erro > 20)
        velocidadePadrao = VELOCIDADE_MINIMA;
    else
        velocidadePadrao = VELOCIDADE_MAXIMA;
}

/*
Função verifica se há cruzamentos na pista
INPUTS: nenhum
OUTPUTS: se houver cruzamento de 4 linhas segue me frente
*/
async Task VerificaCruzamentos(string leituraDeCores)
{
    string ladoDaCurva;

    switch (leituraDeCores)
    {
        case "pppp": // Se todos leem preto, é um cruzamento de 4 linhas, segue em frente por 1 seg e encerra a função
            velocidadePadrao = VELOCIDADE_MEDIA;
            IO.Print("CRUZAMENTO DE 4 LINHAS");
            MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
            await Time.Delay(1000);
            return;
            break;
        case "ppww": // Se for uma curva de 90° para a esquerda, define a variável ladoDaCurva como esquerda
            velocidadePadrao = VELOCIDADE_MEDIA;
            ladoDaCurva = "esquerda";
            break;
        case "wwpp": // Se for uma curva de 90° para a direita, define a variável ladoDaCurva como direita
            velocidadePadrao = VELOCIDADE_MEDIA;
            ladoDaCurva = "direita";
            break;
        default: // Se não for nenhum dos casos, não é um cruzamento, encerra a função
            return;
            break;
    }

    IO.Print($"CURVA DE 90° GRAUS - {ladoDaCurva}"); // Passou no swtich sem encerrar a função = curva; decide o lado da curva de acordo com a variável ladoDaCurva
    // Gira para o lado contrário da curva
    await MotoresPrincipais.CurvaEmGraus((ladoDaCurva == "direita" ? "esquerda" : "direita"), 30, forcaPadrao, velocidadePadrao);
    // Gira para o lado da curva até achar a linha
    while ((ladoDaCurva == "esquerda" ? sensorDeCorR.CorDetectada() : sensorDeCorL.CorDetectada()) != PRETO)
    {
        await Time.Delay(1);
        await MotoresPrincipais.Girar((ladoDaCurva == "direita" ? "direita" : "esquerda"), 0, 0, forcaPadrao, velocidadePadrao);
    }
}

/*
Função verifica se o robô já chegou no ladrilho final
INPUTS: nenhum
OUTPUTS: se já chegou no final, trava os motores
*/
void VerificarLadrilhoFinal(string leituraDeCores)
{
    if (leituraDeCores[0] == 'r' || leituraDeCores[1] == 'r' ||
        leituraDeCores[2] == 'r' || leituraDeCores[3] == 'r')
    {
        MotoresPrincipais.TravaDosMotores(true);
        IO.Print("LADRILHO FINAL");
    }

}
#endregion

#region ObstaculosEVerde
/*
Função verifica se há verde na pista
INPUTS: nenhum
OUTPUTS: se houver verde, executa a curva do verde
*/
async Task VerificarVerde()
{
    int delay = 700;

    string cores = sensorDeCorL.CorDetectada() + sensorDeCorR.CorDetectada();
    switch (cores)
    {
        // Curva de 90° para a esquerda
        case "gw":
        case "gp":
            IO.Print("VERDE - ESQUERDA");
            await Time.Delay(delay);
            await MotoresPrincipais.CurvaEmGraus("esquerda", 45, forcaPadrao, velocidadePadrao);
            break;
        // Curva de 90° para a direita
        case "wg":
        case "pg":
            IO.Print("VERDE - DIREITA");
            await Time.Delay(delay);
            await MotoresPrincipais.CurvaEmGraus("direita", 45, forcaPadrao, velocidadePadrao);
            break;
        // Beco sem saída
        case "gg":
            IO.Print("BECO SEM SAÍDA - DIREITA");
            await MotoresPrincipais.CurvaEmGraus("direita", 175, forcaPadrao, velocidadePadrao);
            break;
    }
}

/*
Função que contém a movimentação do robô para desviar de obstáculos para cada lado
INPUTS: o lado para qual o desvio será feito
OUTPUTS: faz o robô contornar o obstáculo
*/
async Task DesviarDeObstaculo()
{
    // Primeiro Giro
    IO.Print("PRIMEIRO GIRO");
    MotoresPrincipais.Parar();
    await Time.Delay(200);
    await MotoresPrincipais.CurvaEmGraus("direita", 90, forcaPadrao, velocidadePadrao);

    // Primeiro avanço
    IO.Print("PRIMEIRO AVANÇO");
    MotoresPrincipais.Parar();
    await Time.Delay(200);
    MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
    await Time.Delay(1350);

    // Segundo Giro
    IO.Print("SEGUNDO GIRO");
    MotoresPrincipais.Parar();
    await Time.Delay(200);
    await MotoresPrincipais.CurvaEmGraus("esquerda", 90, forcaPadrao, velocidadePadrao);

    // Segundo avanço
    IO.Print("SEGUNDO AVANÇO");
    MotoresPrincipais.Parar();
    await Time.Delay(200);
    await AvancarNoObstaculo(130, "direita");
    if (sensorDeCorL.CorDetectada() != BRANCO) return;

    // Terceiro giro
    IO.Print("TERCEIRO GIRO");
    MotoresPrincipais.Parar();
    await Time.Delay(200);
    await MotoresPrincipais.CurvaEmGraus("esquerda", 90, forcaPadrao, velocidadePadrao);

    // Terceiro avanço
    IO.Print("TERCEIRO AVANÇO");
    await Time.Delay(200);
    await AvancarNoObstaculo(130, "direita");
    if (sensorDeCorL.CorDetectada() != BRANCO) return;

    // Quarta giro
    IO.Print("QUARTO GIRO");
    MotoresPrincipais.Parar();
    await Time.Delay(200);
    await MotoresPrincipais.CurvaEmGraus("esquerda", 90, forcaPadrao, velocidadePadrao);

    // Quarto avanço
    IO.Print("QUARTO AVANÇO");
    await Time.Delay(200);
    await AvancarNoObstaculo(130, "direita");
    if (sensorDeCorL.CorDetectada() != BRANCO) return;
}

/*
Função que realiza os avanços no desvio de obstáculo
*/
async Task AvancarNoObstaculo(int limiteDoContador, string lado)
{
    int contador = 0;
    while (contador < limiteDoContador)
    {
        await Time.Delay(1);
        MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
        if (sensorDeCorL.CorDetectada() == PRETO)
        {
            await VoltarParaALinha(lado);
            break;
        }
        contador++;
        IO.Print(contador.ToString());
    }
}

/*
Função que faz o robô voltar para a linha depois de desviar do obstáculo
*/
async Task VoltarParaALinha(string lado)
{
    IO.Print("VOLTANDO PARA A LINHA");
    MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
    await Time.Delay(250);

    while (sensorDeCorL.CorDetectada() != PRETO)
    {
        await Time.Delay(1);
        await MotoresPrincipais.Girar(lado, 0, 0, forcaPadrao, velocidadePadrao);
    }

}

/*
Função que administra as funções de desvio de obstáculo
INPUTS: nenhum
OUTPUTS: desvia dos obstáculos chamando as funções de movimentação
*/
async Task VerificarObstaculo()
{
    double distanciaIdeal = 2;

    if (ultrassonicoFrontalDireito.LerDistancia() < 1)
    {
        IO.Print("OBSTÁCULO");
        // Aproxima até a distância de desvio ideal
        while (ultrassonicoFrontalDireito.LerDistancia() < distanciaIdeal)
        {
            await Time.Delay(1);

            MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao);
        }

        await DesviarDeObstaculo();
    }

}

/*
Função que verifica se o robô está em uma rampa
*/
async Task VerificarRampa()
{
    const int LIMITE_DO_CONTADOR = 50;
    const int LIMITE_DE_INCLINACAO = 17;

    // Se a inclinação é alta, incrementa o contador
    if (Bot.Inclination > LIMITE_DE_INCLINACAO)
    {
        contadorDaRampa++;
    }

    // Se o contador excede o limite, a rampa é confirmada e o robô segue em linha reta até passar dela
    if (contadorDaRampa > LIMITE_DO_CONTADOR)
    {
        IO.Print("RAMPA");
        giroscopio.RedefinirOffset();

        while (Bot.Inclination > LIMITE_DE_INCLINACAO)
        {
            await Time.Delay(1);
            /* forcaPadrao = 140;
            PIDPrincipal.CalcularErroGiroscopico(giroscopio);
            PIDPrincipal.CalcularPID();
            AplicarPIDAosMotores();
            IO.Print($"PID: {PIDPrincipal.pid} | Direção: {giroscopio.Angulo} | Inclinação: {Bot.Inclination}"); */
            MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
        }
        forcaPadrao = 70;
        contadorDaRampa = 0;
    }

    if ((Utilitarios.Millis() - ultimaAtualizacaoDaRampa) > 1)
    {
        ultimaAtualizacaoDaRampa = Utilitarios.Millis();
        contadorDaRampa = 0;
    }

    if (Bot.Inclination < LIMITE_DE_INCLINACAO)
    {
        contadorDaRampa = 0;
    }
}
#endregion

void imprimirConsole(string leituraDeCores)
{
    if(Utilitarios.Millis() - ultimaAtualizacaoDoConsole >= 0){
        ultimaAtualizacaoDoConsole = Utilitarios.Millis();
        IO.Print($@"
Cores: {leituraDeCores} 
PID: {PIDPrincipal.pid:F1} | Erro: {PIDPrincipal.erro:F1}| Integral: {(PIDPrincipal.Integral != 0 ? "ATIVADO" : "DESATIVADO")}
Velocidade: {velocidadePadrao}
Inclinação: {Bot.Inclination}");
    }
}

#region Resgate
/*
Função que detecta a entrada da área de resgate
*/
async Task VerificarAreaDeResgate(string leituraDeCores)
{
    if (leituraDeCores[0] == 's' || leituraDeCores[1] == 's' ||
        leituraDeCores[2] == 's' || leituraDeCores[3] == 's')
    {
        IO.Print("ÁREA DE RESGATE");
        while (true) { await Time.Delay(1); }
    }
}

/*
Função que define o estado da caçamba
INPUTS: estado -> "levantada" ou "abaixada"
*/
async Task DefinirCacamba(string estado)
{
    double forca = 100;
    double velocidade = 200;
    const double TEMPO_PARA_ABAIXAR = 800;
    const double TEMPO_PARA_LEVANTAR = 600;

    switch (estado)
    {
        case "levantada":
            await servoCacamba.write(forca, velocidade, TEMPO_PARA_LEVANTAR, true);
            break;
        case "abaixada":
            await servoCacamba.write(forca, velocidade, TEMPO_PARA_ABAIXAR, false);
            break;
    }
}

/*
Função que define o estado da garra
INPUTS: estado -> "levantada", "abaixada", "aberta" ou "fechada"
*/
async Task DefinirGarra(string estado)
{
    double forca = 70;
    double velocidade = 200;
    const double TEMPO_PARA_ABAIXAR = 1400;
    const double TEMPO_PARA_LEVANTAR = 1100;
    const double TEMPO_PARA_FECHAR = 1700;
    const double TEMPO_PARA_ABRIR = 500;

    switch (estado)
    {
        case "levantada":
            await servoBracoDaGarra.write(forca, velocidade, TEMPO_PARA_LEVANTAR, true);
            break;
        case "abaixada":
            await servoBracoDaGarra.write(forca, velocidade, TEMPO_PARA_ABAIXAR, false);
            break;
        case "aberta":
            await servoDaGarra.write(forca, velocidade, TEMPO_PARA_ABRIR, false);
            break;
        case "fechada":
            await servoDaGarra.write(forca, velocidade, TEMPO_PARA_FECHAR, true);
            break;
    }
}

#endregion

#region Main
async Task Main()
{
    giroscopio.RedefinirOffset();
    /* caneta1.DefinirCor(0, 225, 0);
    caneta2.DefinirCor(0, 225, 0);
    caneta1.Ligar();
    caneta2.Ligar(); */

    Utilitarios.DefinirInicioDoPrograma();
    MotoresPrincipais.DefinirDadosDeReferencia(90, 1800, 230);
    IO.OpenConsole();
    MotoresPrincipais.TravaDosMotores(false);

    IO.Print(servoBracoDaGarra.Angulo.ToString());

    await DefinirCacamba("abaixada");
    await DefinirGarra("abaixada");
    await DefinirGarra("fechada");
    await DefinirGarra("levantada");

    await MotoresPrincipais.CurvaEmGraus("esquerda", 15, forcaPadrao, velocidadePadrao);

    while (true)
    {
        await Time.Delay(1);

        string leituraDeCores = sensorDeCorLL.CorDetectada() + sensorDeCorL.CorDetectada() + sensorDeCorR.CorDetectada() + sensorDeCorRR.CorDetectada();
        imprimirConsole(leituraDeCores);

        await VerificarObstaculo();
        await VerificarVerde();
        await VerificaCruzamentos(leituraDeCores);
        await VerificarRampa();
        VerificarLadrilhoFinal(leituraDeCores);
        await VerificarAreaDeResgate(leituraDeCores);
        SeguirLinhaComPID();

    }
}
#endregion
#endregion

// ! END MARKUP COMMENT // Program //**************************************************************************************************