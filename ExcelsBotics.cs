const string BRANCO = "w";
const string PRETO = "p";
const string VERDE = "g";
const string VERMELHO = "r";
const string PRATA = "s";
const string UNKNOWN_COLOR = "u";
const int VELOCIDADE_MINIMA = 245;
const int VELOCIDADE_MEDIA = 245;
const int VELOCIDADE_MAXIMA = 245;
int velocidadePadrao = 245;
int forcaPadrao = 100;
double ultimaAtualizacaoDoConsole = 0;
double ultimaAtualizacaoDaRampa = 0;
int contadorDaRampa = 0;
int contadorDaAreaDeResgate = 0;
int vitimasResgatadas = 0;
int repeticoesDeVitimaMorta = 0;
bool vitimaMortaResgatada = false;
bool saiuDaSalaDeResgate = false;

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
        else if (leitura.Red < 95 && leitura.Green > 80 && leitura.Blue > 90 && leitura.Blue < 125)
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
        Mover(0, 0);
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
            this.erro = 500;
        }
        else if (sensor4.CorDetectada() == PRETO)
        {
            this.erro = -500;
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
    public double AnguloBruto => Bot.Compass;

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

        IO.Print($"AJUSTANDO {this.nome} | Trava: {this.motor.Locked}");
        this.motor.Locked = false; await Time.Delay(100);
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

SensorDeCor sensorDeCorCacamba = new SensorDeCor("corCaixaDeVitimas");
SensorDeCor sensorDeCorCacamba2 = new SensorDeCor("corCaixaDeVitimas2");
SensorDeCor sensorDeCorCacamba3 = new SensorDeCor("corCaixaDeVitimas3");
SensorDeCor sensorDeCorFrontal = new SensorDeCor("corFrontal");

SensorUltrassonico ultrassonicoFrontalDireito = new SensorUltrassonico("ultrassonicoFrontalDireito");
SensorUltrassonico ultrassonicoFrontalEsquerdo = new SensorUltrassonico("ultrassonicoFrontalEsquerdo");

SensorUltrassonico ultrassonicoDeResgateDireito = new SensorUltrassonico("ultrassonicoDeResgateDireito");

SensorUltrassonico ultrassonicoEsquerdo = new SensorUltrassonico("ultrassonicoEsquerdo");
SensorUltrassonico ultrassonicoDireito = new SensorUltrassonico("ultrassonicoDireito");

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
    MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao); await Time.Delay(250);
    // Gira para o lado contrário da curva
    await MotoresPrincipais.CurvaEmGraus((ladoDaCurva == "direita" ? "esquerda" : "direita"), 30, forcaPadrao, velocidadePadrao);
    // Gira para o lado da curva até achar a linha
    while ((ladoDaCurva == "esquerda" ? sensorDeCorL.CorDetectada() : sensorDeCorR.CorDetectada()) != PRETO)
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
    int delay = 730;
    int delay2 = 400;

    string cores = sensorDeCorL.CorDetectada() + sensorDeCorR.CorDetectada();
    switch (cores)
    {
        // Curva de 90° para a esquerda
        case "gw":
        case "gp":
            IO.Print("VERDE - ESQUERDA");
            MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
            await Time.Delay(delay);
            await MotoresPrincipais.CurvaEmGraus("esquerda", 90, forcaPadrao, velocidadePadrao);
            MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
            await Time.Delay(delay2);
            break;
        // Curva de 90° para a direita
        case "wg":
        case "pg":
            IO.Print("VERDE - DIREITA");
            MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
            await Time.Delay(delay);
            await MotoresPrincipais.CurvaEmGraus("direita", 90, forcaPadrao, velocidadePadrao);
            MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
            await Time.Delay(delay2);
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
INPUTS: nenhum
OUTPUTS: faz o robô contornar o obstáculo
*/
async Task DesviarDeObstaculo()
{
    double velocidadeDaDireita = velocidadePadrao * 2;
    double velocidadeDaEsquerda = velocidadePadrao * 0.3;

    // Primeiro Giro
    IO.Print("PRIMEIRO GIRO");
    MotoresPrincipais.Parar();
    await Time.Delay(200);
    await MotoresPrincipais.CurvaEmGraus("direita", 90, forcaPadrao, velocidadePadrao);

    // Início do desvio circular
    IO.Print("CONTORNANDO...");
    while (sensorDeCorL.CorDetectada() != PRETO) // Contorna enquanto não houver linha
    {
        await Time.Delay(1);
        MotoresPrincipais.MoverMotoresPorLado(forcaPadrao, velocidadeDaEsquerda, velocidadeDaDireita);
    }

    await VoltarParaALinha("direita");
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
    double distanciaIdeal = 1;

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
    const int LIMITE_DO_CONTADOR = 30;
    int[] LIMITE_DE_INCLINACAO = {50, 355};
    const int LIMITE_CONTADOR_BAIXA_VELOCIDADE = 70;
    int contadorBaixaVelocidade = 0;
    const double LIMITE_DE_ERRO = 50;
    bool houveBaixaVelocidade = false;

    // Se a inclinação é alta, incrementa o contador
    if (Bot.Inclination > LIMITE_DE_INCLINACAO[0] && Bot.Inclination < LIMITE_DE_INCLINACAO[1])
    {
        contadorDaRampa++;
    }

    // Se o contador excede o limite, a rampa é confirmada e o robô segue em linha reta até passar dela
    if (contadorDaRampa > LIMITE_DO_CONTADOR && PIDPrincipal.erro < LIMITE_DE_ERRO) 
    {
        IO.Print($"RAMPA | Velocidade: {Bot.Speed}");
        giroscopio.RedefinirOffset();

        while (Bot.Inclination > LIMITE_DE_INCLINACAO[0] && Bot.Inclination < LIMITE_DE_INCLINACAO[1])
        {
            await Time.Delay(1);
            forcaPadrao = 140;
            /* PIDPrincipal.CalcularErroGiroscopico(giroscopio);
            PIDPrincipal.CalcularPID();
            AplicarPIDAosMotores();
            IO.Print($"PID: {PIDPrincipal.pid} | Direção: {giroscopio.Angulo} | Inclinação: {Bot.Inclination}"); */
            MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);

            // Se o robô estiver parado ou muito lento, aumenta o contador
            if (Bot.Speed < 0.5)
            {
                contadorBaixaVelocidade++;
            }

            // Se a velocidade na rampa for baixa, tenta passar ao contrário
            if (contadorBaixaVelocidade > LIMITE_CONTADOR_BAIXA_VELOCIDADE)
            {
                houveBaixaVelocidade = true;
                forcaPadrao = 200;
                await MotoresPrincipais.CurvaEmGraus("direita", 225, forcaPadrao, velocidadePadrao);
                MotoresPrincipais.TravaDosMotores(true); await DefinirGarra("abaixada"); MotoresPrincipais.TravaDosMotores(false);
                MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao);
                await Time.Delay(8000);
                await MotoresPrincipais.CurvaEmGraus("direita", 205, forcaPadrao, velocidadePadrao);
                contadorBaixaVelocidade = 0;
            }
        }
        if (houveBaixaVelocidade)
        {
            MotoresPrincipais.TravaDosMotores(true);
            await DefinirGarra("levantada");
            await Time.Delay(400);
            MotoresPrincipais.TravaDosMotores(false);
        }
        forcaPadrao = 100;
        contadorDaRampa = 0;
    }

    if ((Utilitarios.Millis() - ultimaAtualizacaoDaRampa) > 1)
    {
        ultimaAtualizacaoDaRampa = Utilitarios.Millis();
        contadorDaRampa = 0;
    }

    if (!(Bot.Inclination > LIMITE_DE_INCLINACAO[0] && Bot.Inclination < LIMITE_DE_INCLINACAO[1]) || PIDPrincipal.erro > LIMITE_DE_ERRO)
    {
        contadorDaRampa = 0;
    }
}
#endregion

#region Resgate
/*
Função que detecta a entrada da área de resgate
*/
async Task VerificarAreaDeResgate(string leituraDeCores)
{
    const double LIMITE_DO_CONTADOR_DO_RESGATE = 5;

    if (leituraDeCores[0] == 's' || leituraDeCores[1] == 's' ||
        leituraDeCores[2] == 's' || leituraDeCores[3] == 's')
    {
        contadorDaAreaDeResgate++;
    }
    else contadorDaAreaDeResgate = 0;

    if (contadorDaAreaDeResgate > LIMITE_DO_CONTADOR_DO_RESGATE)
    {
        IO.Print("ÁREA DE RESGATE");
        MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
        await Time.Delay(2500);
        await ResgateMainLoop();
        forcaPadrao = 100;
    }
}

/*
Função que encapsula o comportamento da área de resgate
*/
async Task ResgateMainLoop()
{
    forcaPadrao = 50;
    while (true)
    {
        await Time.Delay(1);

        if (vitimaMortaResgatada)
        {
            await ProcurarSaida();
            return;
        }

        // Se achou uma vítima
        bool vitimaEncontrada = await ProcurarVitimas();
        bool vitimaCapturada = false;
        string tipoDeVitima = "indefinido";
        if (vitimaEncontrada)
        {
            // Se conseguiu capturar
            vitimaCapturada = await AproximarComVitima();
        }

        if (vitimaCapturada)
        {
            tipoDeVitima = IdentificarVitima();
            IO.Print($"Encontrada: {(vitimaEncontrada ? "sim" : "não")} | Capturada : {(vitimaCapturada ? "sim" : "não")} | Tipo: {tipoDeVitima}");
            await Time.Delay(500);
            bool vitimaDepositada = false;

            while (!vitimaDepositada)
            {
                await Time.Delay(1);
                vitimaDepositada = await DepositarVitima(tipoDeVitima);
            }
        }
    }
}

/*
Função que procura a saída da sala de resgate
*/
async Task ProcurarSaida()
{
    int contadorDeTimeout = 0;
    const int LIMITE_DE_TIMEOUT = 300;

    int contadorDaSaida = 0;
    const int LIMITE_CONTADOR_DA_SAIDA = 4;

    // Procura por um ponto onde o sensor ultrassônico lê distância maior que alcance máximo
    while (!saiuDaSalaDeResgate)
    {
        await Time.Delay(1);
        IO.Print("PROCURANDO A SAÍDA");

        MotoresPrincipais.Girar("direita", 0, 0, forcaPadrao, velocidadePadrao);

        if (ultrassonicoDeResgateDireito.LerDistancia() > 26.5)
        {
            contadorDaSaida++;
        }
        else contadorDaSaida = 0;

        if (contadorDaSaida > LIMITE_CONTADOR_DA_SAIDA)
        {
            IO.Print("SAÍDA ENCONTRADA");
            break;
        }
    }
    MotoresPrincipais.Parar();

    // Vai até a saída e verifica a cor
    while (contadorDeTimeout <= LIMITE_DE_TIMEOUT && !saiuDaSalaDeResgate)
    {
        await Time.Delay(1);

        MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);

        // Se algum detectar prata ou der timeout, cancela, recua a saída e reinicia a busca
        if (sensorDeCorLL.CorDetectada() == PRATA || sensorDeCorL.CorDetectada() == PRATA ||
            sensorDeCorR.CorDetectada() == PRATA || sensorDeCorRR.CorDetectada() == PRATA
            || contadorDeTimeout >= LIMITE_DE_TIMEOUT)
        {
            MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao);
            await Time.Delay(4500);
            await MotoresPrincipais.CurvaEmGraus("direita", 70, forcaPadrao, velocidadePadrao);
            await ProcurarSaida();
            contadorDeTimeout = 0;
        }

        // Se dois ou mais sensores verem preto, entende que achou a saída
        int pretoLL = sensorDeCorLL.CorDetectada() == PRETO ? 1 : 0;
        int pretoL = sensorDeCorL.CorDetectada() == PRETO ? 1 : 0;
        int pretoR = sensorDeCorR.CorDetectada() == PRETO ? 1 : 0;
        int pretoRR = sensorDeCorRR.CorDetectada() == PRETO ? 1 : 0;
        int pretoTotal = pretoLL + pretoL + pretoR + pretoRR;
        if (pretoTotal >= 2)
        {
            MotoresPrincipais.Parar();
            await SairDaSalaDeResgate();
            break;
        }

        IO.Print($"APROXIMANDO DA SAÍDA | Timeout em: {LIMITE_DE_TIMEOUT - contadorDeTimeout} | Contador de preto: {pretoTotal}");

        contadorDeTimeout++;
    }
}

/*
Função auxiliar que faz a saída da sala de resgate
*/
async Task SairDaSalaDeResgate()
{
    const int MARGEM_DE_ERRO_ANGULO = 10;
    // Alinha com o eixo X e vê se é uma parede ou a saída
    int restoDoAngulo = (int)giroscopio.AnguloBruto % 90;
    while (!(restoDoAngulo > -MARGEM_DE_ERRO_ANGULO && restoDoAngulo < MARGEM_DE_ERRO_ANGULO))
    {
        await Time.Delay(1);
        IO.Print($"ALINHANDO COM X | {(int)giroscopio.AnguloBruto % 90}");
        await MotoresPrincipais.Girar("direita", 0, 0, forcaPadrao, velocidadePadrao);
        restoDoAngulo = (int)giroscopio.AnguloBruto % 90;
    }
    MotoresPrincipais.Parar();
    if (ultrassonicoDeResgateDireito.LerDistancia() > 15)
    {
        IO.Print("Saída confirmada em X");
        MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
        await Time.Delay(1000);
        saiuDaSalaDeResgate = true;
        return;
    }
    else
    {
        await MotoresPrincipais.CurvaEmGraus("esquerda", 20, forcaPadrao, velocidadePadrao);
        // Se for parede, alinha com o eixo Y
        restoDoAngulo = (int)giroscopio.AnguloBruto % 90;
        while (!(restoDoAngulo > -MARGEM_DE_ERRO_ANGULO && restoDoAngulo < MARGEM_DE_ERRO_ANGULO))
        {
            await Time.Delay(1);
            IO.Print($"ALINHANDO COM Y | {(int)giroscopio.AnguloBruto % 90}");
            await MotoresPrincipais.Girar("esquerda", 0, 0, forcaPadrao, velocidadePadrao);
            restoDoAngulo = (int)giroscopio.AnguloBruto % 90;
        }
        MotoresPrincipais.Parar();
        if (ultrassonicoDeResgateDireito.LerDistancia() > 15)
        {
            IO.Print("Saída confirmada em Y");
            MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
            await Time.Delay(1000);
            saiuDaSalaDeResgate = true;
            return;
        }
    }
}

/*
Função auxiliar que procura pelo depósito das vítimas
*/
async Task ProcurarDeposito()
{
    const int LIMITE_CONTADOR_DEPOSITO = 10;
    int contadorDoDepostito = 0;
    double distanciaSuperior = 0, distanciaInferior = 0, diferenca = 0;

    // Procura pelo local de depósito comparando ultrassônicos
    MotoresPrincipais.TravaDosMotores(false);
    while (contadorDoDepostito < LIMITE_CONTADOR_DEPOSITO)
    {
        await Time.Delay(1);
        MotoresPrincipais.Girar("direita", 0, 0, forcaPadrao, velocidadePadrao);

        distanciaSuperior = ultrassonicoFrontalDireito.LerDistancia();
        distanciaInferior = ultrassonicoDeResgateDireito.LerDistancia();
        diferenca = distanciaSuperior - distanciaInferior;
        if (diferenca > 3 && diferenca < 6.5)
        {
            contadorDoDepostito++;
        }
        else contadorDoDepostito = 0;

        IO.Print($"Contador do depósito: {contadorDoDepostito}");
        contadorDoDepostito++;
    }

    IO.Print("DEPÓSITO LOCALIZADO");
}

/*
Função que busca pela área de depósito e deposita a vítima
INPUTS: o tipo de vítima: "viva" ou "morta", de preferência retornado pela função IdentificarVitima
OUTPUTS: true se depositou corretamente, false se falhou
*/
async Task<bool> DepositarVitima(string tipo)
{
    string depositoAlvo = tipo == "viva" ? VERDE : VERMELHO;

    await ProcurarDeposito();

    // Vai até o depósito
    while (ultrassonicoDeResgateDireito.LerDistancia() > 0.5)
    {
        await Time.Delay(1);

        MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);

        // Se não for da cor do alvo, cancela e reinicia a busca
        string cor = sensorDeCorFrontal.CorDetectada();
        if (cor != UNKNOWN_COLOR && cor != BRANCO && cor != PRETO && cor != depositoAlvo)
        {
            MotoresPrincipais.Parar(); await Time.Delay(250);
            MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao); await Time.Delay(1000);
            return false;
        }
        // Se estiver perto mas a cor ainda não for a do depósito alvo, cancela e reinicia a busca
        if (ultrassonicoDeResgateDireito.LerDistancia() < 5 && cor != depositoAlvo)
        {
            MotoresPrincipais.Parar(); await Time.Delay(250);
            MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao); await Time.Delay(1000);
            return false;
        }
        // Se estiver apontando para uma saída ou lugar vazio, cancela e reinicia a busca
        if (ultrassonicoDeResgateDireito.LerDistancia() >= 32)
        {
            MotoresPrincipais.Parar(); await Time.Delay(250);
            MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao); await Time.Delay(1000);
            return false;
        }
    }

    // Se não teve retorno antecipado, entende que a aproximação foi concluída com sucesso
    MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
    await Time.Delay(1000);
    MotoresPrincipais.TravaDosMotores(true);
    await DepositarVitima();
    MotoresPrincipais.TravaDosMotores(false);
    
    // Recuar
    MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao);
    await Time.Delay(2500);

    vitimasResgatadas++;
    return true;
}

/*
Função que deposita a vítima (em qualquer lugar)
*/
async Task DepositarVitima()
{
    await DefinirGarra("abaixada pela metade"); await Time.Delay(500);
    await DefinirGarra("aberta"); await Time.Delay(700);
    await DefinirGarra("levantada"); await Time.Delay(700);
    await DefinirGarra("fechada"); await Time.Delay(700);
}

/*
Função que identifica a vítima
INPUTS: nenhum
OUTPUTS: o tipo de vítima (viva ou morta)
*/
string IdentificarVitima()
{
    // Se pelo menos um dos sensores ver preto, é vítima morta
    if (sensorDeCorCacamba.CorDetectada() == PRETO || sensorDeCorCacamba2.CorDetectada() == PRETO || sensorDeCorCacamba3.CorDetectada() == PRETO)
        return "morta";
    // Se todos os três sensores verem branco, é vítima viva
    else if (sensorDeCorCacamba.CorDetectada() == BRANCO && sensorDeCorCacamba2.CorDetectada() == BRANCO && sensorDeCorCacamba3.CorDetectada() == BRANCO)
        return "viva";
    else return "indefinido";
}

/*
Função que faz aproximação para resgate com a vítima encontrada
INPUTS: nenhum
OUTPUTS: true se conseguiu uma vítima, false se ocorreu timeout ou falha na captura
*/
async Task<bool> AproximarComVitima()
{
    const double DISTANCIA_DE_CAPTURA = 0.5;
    const int LIMITE_DE_TIMEOUT = 270;
    int contadorDeTimeout = 0;

    // Posiciona a garra
    MotoresPrincipais.Parar(); MotoresPrincipais.TravaDosMotores(true);
    await DefinirGarra("aberta");
    await DefinirGarra("abaixada");

    // Vai até a vítima
    MotoresPrincipais.TravaDosMotores(false);
    while (ultrassonicoDeResgateDireito.LerDistancia() > DISTANCIA_DE_CAPTURA)
    {
        await Time.Delay(1);
        IO.Print($"Distancia: {ultrassonicoDeResgateDireito.LerDistancia()} | Iterações até timeout: {LIMITE_DE_TIMEOUT - contadorDeTimeout}");
        MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);

        if (contadorDeTimeout > LIMITE_DE_TIMEOUT)
        {
            IO.Print($"CANCELAR RESGATE - TIMEOUT");
            MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao);
            await Time.Delay(3000);
            MotoresPrincipais.TravaDosMotores(true);
            await DefinirGarra("levantada"); await Time.Delay(200);
            await DefinirGarra("fechada");
            MotoresPrincipais.TravaDosMotores(false);
            return false;
        }

        contadorDeTimeout++;
    }

    // Captura a vítima
    await DefinirGarra("fechada"); await Time.Delay(200);
    MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao);
    await Time.Delay(1700);
    MotoresPrincipais.TravaDosMotores(true);
    await DefinirGarra("levantada"); await Time.Delay(200);
    MotoresPrincipais.TravaDosMotores(false);
    vitimaMortaResgatada = IdentificarVitima() == "morta" ? true : false;

    // Se pegou uma morta antes de ter pego as duas vivas, larga e continua procurando
    if (IdentificarVitima() == "morta" && vitimasResgatadas < 2 && repeticoesDeVitimaMorta < 2)
    {
        IO.Print("CANCELAR RESGATE - PRIORIDADE PARA AS VIVAS");
        await MotoresPrincipais.CurvaEmGraus("esquerda", 70, forcaPadrao, velocidadePadrao);
        MotoresPrincipais.Parar(); MotoresPrincipais.TravaDosMotores(true);
        await DefinirGarra("abaixada"); await Time.Delay(500);
        await DefinirGarra("aberta"); await Time.Delay(700);
        await DefinirGarra("levantada"); await Time.Delay(700);
        await DefinirGarra("fechada"); await Time.Delay(700);
        MotoresPrincipais.Parar(); MotoresPrincipais.TravaDosMotores(false);
        await MotoresPrincipais.CurvaEmGraus("direita", 70, forcaPadrao, velocidadePadrao);
        MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao); await Time.Delay(500);
        repeticoesDeVitimaMorta++;
        vitimaMortaResgatada = false;
        return false;
    }

    return sensorDeCorCacamba.CorDetectada() == VERDE ? false : true;
} 

/*
Função que procura as vítimas com base no sensor ultrassônico
INPUT: nenhum
OUTPUT: true se achou uma vítima, false se não achou
*/
async Task<bool> ProcurarVitimas()
{
    const double DISTANCIA_IDEAL_DE_RESGATE = 3;
    // Define o lado da procura
    string ladoDaProcura = "direita";
    string ladoOpostoAProcura = "esquerda";
    
    // Inicia a busca pelas vítimas
    double distancia = ultrassonicoDeResgateDireito.LerDistancia(), distanciaAnterior = distancia;
    string cor = sensorDeCorFrontal.CorDetectada();
    
    // Busca pela vítima se baseando em quedas bruscas na distância
    while (true)
    {
        await Time.Delay(1);
        MotoresPrincipais.Girar(ladoDaProcura, 0, 0, forcaPadrao, velocidadePadrao);
        distancia = ultrassonicoDeResgateDireito.LerDistancia();
        cor = sensorDeCorFrontal.CorDetectada();

        double percentual = (distancia / distanciaAnterior) * 100;
        // se houve queda brusca na distância ou o sensor leu preto (vitima morta), identifica como vítima
        if ((percentual > 2 && percentual < 70) || cor == PRETO)
        {
            // Se a distância anterior for 32 (fora de alcance) e a diferença entre os sensores superiores e inferiores for baixa, entende que não é uma vítima (e sim uma parede) e passa para a próxima iteração
            if (distanciaAnterior == 32 && ultrassonicoFrontalDireito.LerDistancia() - ultrassonicoDeResgateDireito.LerDistancia() < 3)
            {
                continue;
            }

            IO.Print($"VITIMA ENCONTRADA | Distancia: {distancia} | Percentual: {percentual}");
            MotoresPrincipais.CurvaEmGraus("esquerda", 5, forcaPadrao, velocidadePadrao);
            while (ultrassonicoDeResgateDireito.LerDistancia() < DISTANCIA_IDEAL_DE_RESGATE)
            {
                await Time.Delay(1);
                MotoresPrincipais.Mover(forcaPadrao, -velocidadePadrao);
            }
            MotoresPrincipais.Parar();
            return true;
        }

        IO.Print($"D: {distancia} | P: {percentual}");
        distanciaAnterior = distancia;
    }

    return false;
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
    double forca = 200;
    double velocidade = 200;
    const double TEMPO_PARA_ABAIXAR = 1400;
    const double TEMPO_PARA_LEVANTAR = 1250;
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
            await servoDaGarra.write(forca * 1.5, velocidade, TEMPO_PARA_FECHAR, true);
            break;
        case "abaixada pela metade":
            await servoBracoDaGarra.write(forca, velocidade, TEMPO_PARA_ABAIXAR / 2, false);
            break;
    }
}

#endregion

#region Main
void imprimirConsole(string leituraDeCores)
{
    if(Utilitarios.Millis() - ultimaAtualizacaoDoConsole >= 1){
        ultimaAtualizacaoDoConsole = Utilitarios.Millis();
        IO.Print($@"
Cores: {leituraDeCores} 
PID: {PIDPrincipal.pid:F1} | Erro: {PIDPrincipal.erro:F1} | Integral: {(PIDPrincipal.Integral != 0 ? "ATIVADO" : "DESATIVADO")}
Velocidade: {Bot.Speed}
Inclinação: {Bot.Inclination} | Contador da Rampa: {contadorDaRampa}");
    }
}

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
    await DefinirGarra("fechada");
    await DefinirGarra("levantada");

    MotoresPrincipais.Mover(forcaPadrao, velocidadePadrao);
    await Time.Delay(500);

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