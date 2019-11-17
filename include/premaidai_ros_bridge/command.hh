#ifndef _PREMAIDAI_COMMAND_HH_
#define _PREMAIDAI_COMMAND_HH_

#include <vector>
#include <cstdint>
#include "premaidai_ros_bridge/common.hh"

namespace premaidai_ros_bridge
{
    typedef std::vector<uint8_t> FrameData;

    /**
     * @brief Commandの種類を表現するためのenum
     */
    enum class CommandType
    {
        GET_PARAM = 0x01,       // プリメイドAIの情報を取得する（サーボ、バッテリー等）
        STOP = 0x05,            // 強制停止する
        SET_SERVO_POS= 0x18,    // サーボの位置を設定する
        SET_SERVO_PARAM = 0x19, // サーボのパラメータを設定する
        SET_LED_CMD = 0x1E,     // 胸のLEDを制御する
        PLAY_MOTION = 0x1F,     // プリセットのMotionを再生する
    };

    /**
     * @brief Commandのベースクラス
     */
    class Command
    {
    public:
        /**
         * @brief Construct a new Command object
         *
         * @param type : コマンドタイプ（CommandType classから選択）
         */
        explicit Command(const CommandType& type);

        /**
         * @brief Destroy the Command object
         */
        virtual ~Command();

        /**
         * @brief validate packet parity
         */
        bool validatePacket();

        /**
         * @brief print packet information
         */
        void printPacket();

    protected:
        FrameData data() const;
        void data(const FrameData& data);
        CommandType type() const;

    private:
        FrameData data_;        /** 管理データ */
        CommandType type_;      /** Command type */
    };


    /**
     * @brief 送信コマンドのベースクラス
     */
    class SendCommand : public Command
    {
    public:
        explicit SendCommand(const CommandType& type);
        virtual ~SendCommand();
        FrameData getPacket() const;

    protected:
        /**
         * @brief 送信パケットを構築する
         */
        FrameData finalizePacket(const FrameData& data) const;
    };


    /**
     * @brief プリセットMotionの種類
     */
    enum class MotionType
    {
        DANCE = 1,           // ダンス再生
        HOME = 61,           // ホーム
        DOZO = 62,           // どうぞ
        WAKUWAKU = 63,       // ワクワク
        KOSHINITE = 64,      // コシニテ
        YOUKOSO = 65,        // ようこそ
        ONEGAI = 66,         // おねがい
        BYEBYE = 67,         // バイバイ
        RKISS = 68,          // 右にキス
        LKISS = 69,          // 左にキス
        GUN = 70,            // ピストル
        KEIREI = 71,         // 敬礼
        EN = 72,             // え〜ん
        HART = 73,           // ハート
        KIRA = 74,           // キラッ
        ANATAHE = 75,        // あなたへ
        MONKEY = 76,         // もんきー
        GOGO = 77,           // Gox2
        GUITER = 78,         // エアギター
        RTURN = 79,          // 右ターン
        LTURN = 80           // 左ターン
    };


    /**
     * @brief Motion command generator class
     */
    class MotionRequestCommand : public SendCommand
    {
    public:
        explicit MotionRequestCommand(const MotionType& motion);
        virtual ~MotionRequestCommand();
    };

    /**
     * @brief Servo status request command generator class
     */
    class ServoStatusRequestCommand : public SendCommand
    {
    public:
        ServoStatusRequestCommand(unsigned int start_id, unsigned int servo_num);
        virtual ~ServoStatusRequestCommand();
    };

    /**
     * @brief Servo off requeset command class
     * 本コマンド実行後はSetPos系のコマンドを実行できないので注意
     */
    class ServoOffRequestCommand : public SendCommand
    {
    public:
        ServoOffRequestCommand();
        virtual ~ServoOffRequestCommand();
    };

    /**
     * @brief ServoControlData
     */
    struct ServoControlData
    {
        uint8_t id;
        uint16_t position;
    };
    typedef std::vector<ServoControlData> ServoControlDataArray;

    /**
     * @brief Set servo target position command class
     */
    class ServoControlCommand : public SendCommand
    {
    public:
        // ServoControlCommand(const DoubleArray& angles, const JointConfigMap& config);
        explicit ServoControlCommand(const ServoControlDataArray& data_array);
        virtual ~ServoControlCommand();
    };

    /**
     * @brief 受信コマンドのベースクラス
     */
    class ReceiveCommand : public Command
    {
    public:
        explicit ReceiveCommand(const CommandType& type, const FrameData& packet);
        virtual ~ReceiveCommand();
        bool accepted() const;

    protected:
        virtual bool unpack();
        void accepted(bool accept);

    private:
        bool is_accepted_;
    };

    /**
     * @brief Motion response command の動作解析
     */
    class MotionResponseCommand : public ReceiveCommand
    {
    public:
        explicit MotionResponseCommand(const FrameData& data);
        virtual ~MotionResponseCommand();
    };


    struct ServoStatus
    {
        bool enable;
        uint8_t id;
        uint16_t actual_position;
        uint16_t offset;
        uint16_t command_position;
        uint16_t gyro_direction;
        uint16_t gyro_scale;
        uint8_t speed;
        uint8_t stretch;
    };

    typedef std::vector<ServoStatus> ServoStatusArray;

    /**
     * @brief servo status command の解析コマンド
     */
    class ServoStatusResponseCommand : public ReceiveCommand
    {
    public:
        explicit ServoStatusResponseCommand(const FrameData& data);
        virtual ~ServoStatusResponseCommand();
        ServoStatusArray getServoStatus() const;

    protected:
        virtual bool unpack();

    private:
        ServoStatusArray servo_status_;
    };


    class ServoControlResponseCommand : public ReceiveCommand
    {
    public:
        explicit ServoControlResponseCommand(const FrameData& data);
        virtual ~ServoControlResponseCommand();
    };

}; // end namespae premaidai_ros_bridge

#endif // _PREMAIDAI_COMMAND_HH_