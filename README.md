## Проект модуля сбора телеметрии и управления круиз-контролем грузовика

Модуль подключается к приложению Android по каналу Bluetooth. Модуль на базе ESP32 (стандартный объем памяти) и внешнего контроллера CAN MCP2515. К модулю опционально подключаются модули расширения (CAN, LIN, Analog) для управления круиз-контролем. Тип определяется автоматически при подключении модуля. Реализован механизм защиты от клонирования.

Лицевой вид платы и вид в корпусе.
![image](https://github.com/user-attachments/assets/40b04efe-0a1a-44da-9eb1-6bd556c1dcd9)
![image](https://github.com/user-attachments/assets/1f67fdbc-0046-40d7-82e2-fbc2607bbada)


### Функционал:

1. **Телеметрия:**
   - Автоматический выбор скорости CAN шины и типа ID, детекция обрыва одного из проводов дифференциальной пары;
   - Фильтрация входящих сообщений шины CAN (динамическое конфигурирование);
   - Распарсивание CAN сообщений с поддержкой разных типов данных (динамическое конфигурирование);
   - Использование суммарных счетчиков для параметров (динамическое конфигурирование);
   - Отдельные каналы для связи с Android приложением (управляются с Android):
     - Канал для извлеченных значений;
     - Канал RAW потока сообщений (мост);
     - Канал связанных отфильтрованных сообщений (обработка на стороне Android);
     - Канал потока данных транспортного протокола;
     - Канал инжекции сообщений в шину CAN (переход в активный режим на время трансляции).

2. **Bluetooth (Classic 4.0):**
   - Работа через кастомный легковесный бинарный протокол;
   - Контроль версий, канал обновления прошивки модуля по воздуху, детекция ошибок;
   - Защита подключения пин-кодом.
  
3. **Управление круиз-контролем:**
   - Режим виртуализации шины CAN1 (подключение всего функционала для работы параллельно с телеметрическим блоком);
   - Модуль CAN2 - использование встроенного CAN контроллера;
   - Модуль LIN - использование встроенного UART и таймеров (низкоуровневый кастомный контроллер LIN с функцией подмены сообщений);
   - Модуль Analog - использование АЦП и логических уровней;
   - Конфигурирование;
   - Режим прослушивания сети;
   - Режим инжекции в сеть предустановленных сообщений для управления круиз-контролем;
   - Отслеживание целевых (назначается) значений в шине для стабильной работы модуля.
  
4. **Хранение:**
   - Сохранение конфигурации в флэш;
   - Опциональное сохранение извлеченной телеметрии на внешнюю SD карту (Черный ящик).
 
5. **Логирование:**
   - Важные действия и критические ошибки всего проекта логируются и отправляются на Android устройство.

### Участки кода приложения на C

#### Обработка CAN-сообщений с использованием транспортного протокола J1939 и фильтрацией

```c
if (mMCP2515_RX->RXBnDLC.DLC == 8) {
    timCanBusActive = xTaskGetTickCount();
    cntRxCAN1msg++;
    
    // Обработка транспортного протокола J1939 (TP)
    uint32_t pgn = (mMCP2515_RX->RXBn_SID_EID_not_norm.all & CAN1939_TP_ALL_MASK);
    if (pgn == CAN1939_TP_BAM_PGN) {
        check_BAM_cmd((struct transport_bum_str *) &mMCP2515_RX->data0, mMCP2515_RX->RXBn_SID_EID_not_norm.RXBnSIDH);
    } else if (pgn == CAN1939_TP_DATA_PGN) {
        setNewTP_data(&mMCP2515_RX->data0, mMCP2515_RX->RXBn_SID_EID_not_norm.RXBnSIDH);
    } else if (Cmd.sys.Status.B.sys_InitFilter != 0) {
        filterMainStr *comparedFilter = binarySearch_mut(mMCP2515_RX->RXBn_SID_EID_not_norm.all);
        if (comparedFilter != 0) {
            cmdCANreqest.B.CAN1_wiresConnectFault = 0;

            comparedFilter->data.u32[0] = mMCP2515_RX->data32_0;
            comparedFilter->data.u32[1] = mMCP2515_RX->data32_1;

            uint32_t time = xTaskGetTickCount();
            comparedFilter->timestamp = time;

            // Передача номера фильтра на обработку полученных CAN данных и вычисление переменных
            if (xQueueSend(xQueue_can_rx, &comparedFilter, (TickType_t) 1) != pdTRUE) {
                tprintf("CAN1 ERR put\n");
            } 
        }
    }
}
```

#### Функция бинарного поиска для фильтрации сообщений CAN

```c
filterMainStr *binarySearch_mut(uint32_t rawId) {
    if (xSemaphoreTake(xSemaphore, 1) == pdFALSE)
        return 0;
    int left = 0;
    int right = compare_filter_counter - 1;
    int middle;
    filterMainStr *fiter = 0;
    rawId &= RAW_id_mask_for_PRIORITY;
    
    while (left <= right) {
        middle = left + (right - left) / 2;

        if (arrIdForCompare->array[middle].rawId == rawId) {
            fiter = arrIdForCompare->array[middle].fiter;
            break;
        }

        if (arrIdForCompare->array[middle].rawId > rawId) {
            right = middle - 1;
        } else {
            left = middle + 1;
        }
    }
    xSemaphoreGive(xSemaphore);
    return fiter;
}
```

#### Обработка транспортного протокола J1939

**Инициализация приёма данных:**

```c
void check_BAM_cmd(void *msg1, uint8_t src) {
    struct transport_bum_str *msg = msg1;
    if (msg->const_20 == 0x20) {
        transport_str *freeSlot = getFreeSlot(src);
        if (freeSlot == 0) {
            if (((slotBeasyCount++) & 0x1F) == 0) {
                addLogsString("rx TP: Err - no slot\n");
                printf("rx TP: Err - no slot\n");
            }
        } else {
            if (msg->bytes_size > MAX_TRANSPORT_DATA_RX) {
                return;
            }
            freeSlot->bum.u32[0] = msg->u32[0];
            freeSlot->bum.u32[1] = msg->u32[1];
            freeSlot->curTp_msgSrc = src;
            freeSlot->ttpDataSlots->waitForTx = 2;
            freeSlot->bytes_cnt = 0;
            freeSlot->packs_cnt = 0;
            freeSlot->msg.pgn = msg->pgn;
        }
    } else {
        printf("rx TP_BAM src %02X Pgn %04X not start; First byte %02X != 0x20\n", src, msg->pgn, msg->const_20);
    }
}
```

**Процесс приёма данных:**

```c
void setNewTP_dataToSource(const uint8_t *data, transport_str *tp) {
    if (tp->bum.const_20 == 0x20 && data[0] == (++tp->packs_cnt)) {
        if (tp->packs_cnt == tp->bum.number_of_packets) {
            for (int k = 1; tp->bytes_cnt < tp->bum.bytes_size; tp->bytes_cnt++, k++) {
                tp->msg.arr[tp->bytes_cnt] = data[k];
            }
            putValueDataForSend(tp);
            tp->bum.const_20 = 0;
        } else {
            for (int k = 1; k < 8; tp->bytes_cnt++, k++) {
                tp->msg.arr[tp->bytes_cnt] = data[k];
            }
        }
    } else {
        tp->bum.const_20 = 0;
        tp->curTp_msgSrc = 0xff; // reserved value
        tp->packs_cnt = 0;
        tp->ttpDataSlots->waitForTx = 0;
        if (((tpMessagesLost++) & 0x1F) == 0) {
            printToLog("rx TP: Err - lost %d\n", tpMessagesLost);
            printf("rx TP: Err - lost %d\n", tpMessagesLost);
        }
    }
}
```

#### Обработка прерываний UART для LIN

```c
void IRAM_ATTR uart_intr_handle(void *arg) {
    uint16_t rx_fifo_len;
    uint32_t status = LIN_UART->int_st.val; // чтение статуса прерывания UART

    if (status & (UART_LL_INTR_MASK ^ UART_INTR_RXFIFO_FULL)) {
        LIN_UART_TOUT();
        dataPoint = 0;
        if (ext_board.lin.tx.pinControlSet == 1)

 {
            ext_board.lin.tx.pinControlSet = 2;
            switchTimersAndUartToRegularWork();
        } else {
            gpio_set_level(GPIO_CRUIZE_LIN_terminator, 0); // активный режим slave
            ext_board.lin.tx.pinControlSet = 0;
        }
    } else if (status & (UART_RXFIFO_FULL_INT_CLR_M)) {
        rx_fifo_len = LIN_UART->status.rxfifo_cnt; // чтение количества байт в буфере UART
        uint8_t c;
        while (rx_fifo_len != 0) {
            c = (uint8_t) LIN_UART->fifo.rw_byte;
            if (ext_board.lin.tx.pinControlSet == 0) {
                LIN_UART_compil(c, rx_fifo_len > 1); // чтение всех байт
            }
            rx_fifo_len--;
        }
    }
    uart_clear_intr_status(LIN_UART_NUM, status);
}
```

#### Обработка прерываний LIN шины с детекцией скорости и управлением состояниями

```c
void gpio_isr_handlerLIN(void *arg) {
    if ((uint32_t)arg == GPIO_CRUIZE_LINrx) {
        switch (ext_board.lin.rx.switchLIN) {
            case (switchLIN_Idle): { 
                setLINpinInterruptMode(GPIO_INTR_NEGEDGE);
                ext_board.lin.rx.switchLIN = switchLIN_WaitCatchBreakCond;
                bit_compare_counter = 0;
                ext_board.lin.rx.lin_bit_time_mks = 0;
                ext_board.lin.rx.init.setUart = 0;
                ext_board.lin.rx.state = StartExtBoard_State_None;
            } break;

            case (switchLIN_WaitCatchBreakCond): {
                timer_set_counter_value(LINtimersCoreGroup, TIMER_0, 0x00000000ULL);
                timer_start(LINtimersCoreGroup, TIMER_0);
                setLINpinInterruptMode(GPIO_INTR_POSEDGE);
                ext_board.lin.rx.switchLIN = switchLIN_CatchTime13Bits;
            } break;

            case (switchLIN_CatchTime13Bits): {
                if (timer_get_counter_value(LINtimersCoreGroup, TIMER_0, &tim_val_on_pin_isr) == ESP_OK) {
                    if ((tim_val_on_pin_isr > 100 && tim_val_on_pin_isr < 3000) && (ext_board.lin.rx.lin_bit_time_mks < tim_val_on_pin_isr)) {
                        if (bit_compare_counter > 10) {
                            ext_board.lin.rx.lin_bit_time_mks = tim_val_on_pin_isr;
                        }
                    }

                    if ((++bit_compare_counter) > 60) {
                        ext_board.lin.rx.switchLIN = switchLIN_WorkedState;
                        ext_board.lin.rx.lin_bit_time_mks = (ext_board.lin.rx.lin_bit_time_mks / 13);
                        ext_board.lin.rx.state = StartExtBoard_CatchSpeed;
                        setLINpinInterruptMode(GPIO_INTR_DISABLE);
                        timer_pause(LINtimersCoreGroup, TIMER_0);
                        ext_board.lin.rx.init.getBitTime = 1;
                    } else {
                        ext_board.lin.rx.switchLIN = switchLIN_WaitCatchBreakCond;
                        setLINpinInterruptMode(GPIO_INTR_NEGEDGE);
                    }
                } else {
                    ext_board.lin.rx.switchLIN = switchLIN_Idle;
                }
            } break;

            default: {
                setLINpinInterruptMode(GPIO_INTR_DISABLE);
                uart_clear_intr_status(LIN_UART_NUM, UART_LL_INTR_MASK);
                uart_enable_intr_mask(LIN_UART_NUM, UART_INTR_CONFIG_FLAG);
            } break;
        }
    } else {
        tprintf("ISR LIN wrong: \n");
        addLogsString("ISR LIN wrong\n");
    }
}
```

#### Управление и отправка извлеченных значений DBC

```c
void sendDirect(void *pnt) {
    valueStr *val = (valueStr *) pnt;
	
	if (val == 0) return;
	
    if (val->dataType == dataTypeString) {
        putDbcToSend(pnt);
        return;
    } else {
        int size = proc_cmdCANconfTX_D_Direct(tmpMsg.packet.msg.data, val);
        tmpMsg.packet.msg.len = size;
        tmpMsg.packetLen = makeBtMsgContainer128(&tmpMsg);
        setSendToBtDelay(&tmpMsg, 3);
    }
}
```

**Извлечение из очереди указателя на структуру для отправки:**
Реализована буферизация данных для пакетной отправки.
```c
void processIncomingData() {
    DbcDataBuffer *newData = &dbcBuffers[sendDBC.Sv];
    receiveCANData(newData->data, &newData->len, &newData->type);  // Получаем данные
    
    sendDirect(newData);  // Отправляем данные на обработку и добавляем в очередь
}

void sendDirect(void *pnt) {
    valueStr *val = (valueStr *) pnt;
    
    if (val == 0) return;
    
    if (val->dataType == dataTypeString) {
        putDbcToSend(pnt);
        return;
    } else {
        int size = proc_cmdCANconfTX_D_Direct(tmpMsg.packet.msg.data, val); // обработка сообщения с ключом 0xD
        tmpMsg.packet.msg.len = size;
        tmpMsg.packetLen = makeBtMsgContainer128(&tmpMsg);
        setSendToBtDelay(&tmpMsg, 3);
    }
}

void sendProcessedData() {
    void *dataToSend;
    
    while ((dataToSend = getDbcToSend()) != 0) {
        if (sendToBluetooth(dataToSend) != SUCCESS) {
            logError("Failed to send data via Bluetooth");
        } else {
            logInfo("Data sent successfully via Bluetooth");
        }
    }
}
```

---
