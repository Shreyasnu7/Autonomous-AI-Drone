import 'package:flutter/material.dart';
import 'log_detail_dialog.dart';

class HangarLogTile extends StatelessWidget {
  final String date;
  final String loc;
  final String dur;
  final String status;
  final String filePath;

  const HangarLogTile({
    Key? key,
    required this.date,
    required this.loc,
    required this.dur,
    required this.status,
    required this.filePath,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: () {
        showDialog(
          context: context, 
          builder: (_) => LogDetailDialog(
            date: date, 
            loc: loc, 
            dur: dur, 
            status: status, 
            filePath: filePath
          )
        );
      },
      child: Container(
        margin: const EdgeInsets.only(bottom: 8),
        padding: const EdgeInsets.all(12),
        decoration: BoxDecoration(color: Colors.white.withOpacity(0.05), borderRadius: BorderRadius.circular(8)),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Row(children: [
              Icon(Icons.flight, color: status == "SUCCESS" ? Colors.green : Colors.amber, size: 16),
              const SizedBox(width: 10),
              Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                Text(loc, style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 14, color: Colors.white)), 
                Text(date, style: const TextStyle(color: Colors.white38, fontSize: 10))
              ])
            ]),
            Text(dur, style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 14, color: Colors.white))
          ]
        ),
      ),
    );
  }
}
